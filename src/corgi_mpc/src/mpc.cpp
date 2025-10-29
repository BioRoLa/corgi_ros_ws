#include "mpc.hpp"
#include <stdexcept>

void ModelPredictiveController::load_config() {
    YAML::Node config;

    const char* home_path = std::getenv("HOME");
    if (!home_path) {
        throw std::runtime_error("HOME environment variable not set");
    }
    std::string config_file_path = std::string(home_path) + "/corgi_ws/corgi_ros_ws/src/corgi_mpc/config/config.yaml";
    config = YAML::LoadFile(config_file_path);

    std::vector<double> diag = config["Q_diagonal"].as<std::vector<double>>();
    if (diag.size() != n_x) {
        throw std::runtime_error("Q_diagonal size mismatch with n_x");
    }

    Q = Eigen::MatrixXd::Zero(n_x, n_x);
    for (int i = 0; i < n_x; ++i) {
        Q(i, i) = diag[i];
    }
}

void ModelPredictiveController::init_matrices(const double *ra, const double *rb, const double *rc, const double *rd) {
    // Dynamics matrices (Ac, Bc) initialization
    Ac.setZero();
    Ac(0, 6) = 1.0;
    Ac(1, 7) = 1.0;
    // Ac(2, 8) = 1.0;
    Ac(3, 9) = 1.0;
    // Ac(4, 10) = 1.0;
    Ac(5, 11) = 1.0;
    Ac(11, 12) = 1.0;  // gravity

    // Compute the *actual* inertia
    double Ixx = m*(l*l + h*h)/12.0;
    double Iyy = m*(w*w + h*h)/12.0;
    double Izz = m*(w*w + l*l)/12.0;

    // Build body‐frame inertia and invert once
    Eigen::Matrix3d BI_body;
    BI_body << Ixx, 0,   0,
               0,   Iyy, 0,
               0,   0,   Izz;
    Eigen::Matrix3d Iinv = BI_body.inverse();

    // Fill Bc with I^{-1} [r]_x
    Bc.setZero();
    std::array<const double*,4> rs = {ra, rb, rc, rd};
    for(int i = 0; i < 4; ++i) {
        Eigen::Vector3d ri(rs[i][0], rs[i][1], rs[i][2]);
        Eigen::Matrix3d r_skew;
        r_skew << 0, -ri.z(), ri.y(),
                  ri.z(), 0, -ri.x(),
                  -ri.y(), ri.x(), 0;
        Bc.block<3,3>(6,  3*i) = Iinv * r_skew;
        Bc.block<3,3>(9,  3*i) = Eigen::Matrix3d::Identity() / m;
    }

    // Discretize the system using ZOH
    Eigen::MatrixXd M(n_x + n_u, n_x + n_u);
    M.setZero();
    M.topLeftCorner(n_x, n_x) = Ac;
    M.topRightCorner(n_x, n_u) = Bc;
    Eigen::MatrixXd expM = (M * dt).exp();
    Ad = expM.topLeftCorner(n_x, n_x);
    Bd = expM.topRightCorner(n_x, n_u);

    // Cost matrices
    // Q = Eigen::MatrixXd::Zero(n_x, n_x);
    // Q.diagonal() << 30,    30,    0,     // roll, pitch, yaw
    //                 0,     0,     300,   // x, y, z
    //                 1e-1,  1e-1,  0,     // ω_x, ω_y, ω_z
    //                 0,     0,     5,     // v_x, v_y, v_z
    //                 0;                   // additional state

    R = 1e-8 * Eigen::MatrixXd::Identity(n_u, n_u);
}

Eigen::VectorXd ModelPredictiveController::step(const Eigen::VectorXd &x, const Eigen::VectorXd &x_ref,
                                                const bool *selection_matrix, std::vector<corgi_msgs::msg::ForceState*> force_state_modules) {
    // Build prediction matrices
    Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero((N - 1) * n_x, n_x);
    Eigen::MatrixXd B_qp = Eigen::MatrixXd::Zero((N - 1) * n_x, (N - 1) * n_u);

    std::vector<Eigen::MatrixXd> A_pow(N);
    A_pow[0] = Eigen::MatrixXd::Identity(n_x, n_x);
    for(int k = 1; k <= N-1; ++k) {
        A_pow[k] = Ad * A_pow[k-1];
    }

    for(int i = 0; i < N-1; ++i){
        A_qp.block(i*n_x, 0, n_x, n_x) = A_pow[i+1];
        for(int j = 0; j <= i; ++j){
            B_qp.block(i*n_x, j*n_u, n_x, n_u) = A_pow[i-j] * Bd;
        }
    }


    // Cost matrices
    Eigen::MatrixXd Q_N = Eigen::MatrixXd::Zero((N - 1) * n_x, (N - 1) * n_x);
    for (int i = 0; i < N - 1; ++i) Q_N.block(i * n_x, i * n_x, n_x, n_x) = Q;

    Eigen::MatrixXd R_N = Eigen::MatrixXd::Zero((N - 1) * n_u, (N - 1) * n_u);
    for (int i = 0; i < N - 1; ++i) R_N.block(i * n_u, i * n_u, n_u, n_u) = R;

    if (Q_N.rows() != B_qp.rows() || Q_N.cols() != B_qp.rows()) {
        throw std::runtime_error("Q_N and B_qp size mismatch!");
    }
    if (R_N.rows() != B_qp.cols() || R_N.cols() != B_qp.cols()) {
        throw std::runtime_error("R_N and B_qp size mismatch!");
    }
    
    Eigen::MatrixXd H = 2 * (B_qp.transpose() * Q_N * B_qp + R_N);
    Eigen::VectorXd g = 2 * B_qp.transpose() * Q_N * (A_qp * x - x_ref);
    H += 1e-6 * Eigen::MatrixXd::Identity(H.rows(), H.cols());
    

    // Constraints
    const int constraint_per_step = (3 + 3 + 0 + 1) * 4;  // bounds(3) + selection(3) + friction(1) + fy(1)
    const int total_constraints = (N - 1) * constraint_per_step;

    Eigen::SparseMatrix<double> constraints(total_constraints, (N - 1) * n_u);
    Eigen::VectorXd lower_bound(total_constraints);
    Eigen::VectorXd upper_bound(total_constraints);

    int row = 0;
    for (int k = 0; k < (N - 1); ++k) {
        for (int foot = 0; foot < 4; ++foot) {
            int base_idx = k * n_u + foot * 3;

            // 1. Force bounds
            constraints.insert(row, base_idx) = 1.0;
            lower_bound(row) = fx_lower_bound;
            upper_bound(row) = fx_upper_bound; ++row;

            constraints.insert(row, base_idx + 1) = 1.0;
            lower_bound(row) = -1e-6; upper_bound(row) = 1e-6; ++row;

            constraints.insert(row, base_idx + 2) = 1.0;
            lower_bound(row) = fz_lower_bound;
            upper_bound(row) = fz_upper_bound; ++row;

            // 2. Stance selection
            double s = selection_matrix[foot] ? 0.0 : 1.0;
            for (int i = 0; i < 3; ++i) {
                constraints.insert(row, base_idx + i) = s;
                lower_bound(row) = 0.0;
                upper_bound(row) = 0.0; ++row;
            }

            // 3. Friction cone
            // constraints.insert(row, base_idx) = 1.0;
            // lower_bound(row) = -friction_coef*abs(force_state_modules[foot]->Fy);
            // upper_bound(row) = -friction_coef*abs(force_state_modules[foot]->Fy); ++row;

            // 4. fy = 0
            constraints.insert(row, base_idx + 1) = 1.0;
            lower_bound(row) = 0.0; upper_bound(row) = 0.0; ++row;
        }
    }
    constraints.makeCompressed();

    // OSQP Solver Setup
    OsqpEigen::Solver solver;
    solver.data()->setNumberOfVariables((N - 1) * n_u);
    solver.data()->setNumberOfConstraints(total_constraints);

    Eigen::SparseMatrix<double> H_sparse(H.rows(), H.cols());
    H_sparse = H.sparseView();
    H_sparse.makeCompressed();
    solver.data()->setHessianMatrix(H_sparse);

    solver.data()->setGradient(g);
    solver.data()->setLinearConstraintsMatrix(constraints);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);

    solver.settings()->setVerbosity(false);
    // solver.settings()->setWarmStart(true);
    // solver.settings()->setPolish(false);

    if (!solver.initSolver()) throw std::runtime_error("[OSQP] Initialization failed.");
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        throw std::runtime_error("[OSQP] Solver failed to converge.");

    const Eigen::VectorXd& u_opt = solver.getSolution();
    return u_opt.head(n_u);
}

void check_contact_state(int swing_leg, std::vector<corgi_msgs::msg::ContactState*> contact_state_modules){
    switch (swing_leg) {
        case 0:
            contact_state_modules[0]->contact = false;
            contact_state_modules[1]->contact = true;
            contact_state_modules[2]->contact = false;
            contact_state_modules[3]->contact = true;
            break;
        case 1:
            contact_state_modules[0]->contact = true;
            contact_state_modules[1]->contact = false;
            contact_state_modules[2]->contact = true;
            contact_state_modules[3]->contact = false;
            break;
        case 2:
            contact_state_modules[0]->contact = false;
            contact_state_modules[1]->contact = true;
            contact_state_modules[2]->contact = false;
            contact_state_modules[3]->contact = true;
            break;
        case 3:
            contact_state_modules[0]->contact = true;
            contact_state_modules[1]->contact = false;
            contact_state_modules[2]->contact = true;
            contact_state_modules[3]->contact = false;
            break;
        default:
            contact_state_modules[0]->contact = false;
            contact_state_modules[1]->contact = false;
            contact_state_modules[2]->contact = false;
            contact_state_modules[3]->contact = false;
            break;
    }
        
}