#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


#define SMALL_NUMBER 0.001
/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  // Set to true only when the initialization is completed.
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = MatrixXd::Zero(5, 5);

  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // time when the state is true, in us
  time_us_ = 0; 

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.7;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // Set state dimension to 5.
  // These are px, py, speed(v), yaw(psi), yaw_rate(psi')
  // Refer Lession 8, Section 3 (The CTRV Model State Vector).
  n_x_ = x_.size();

  // Set augmented state demension.
  // In the augmented state, the noise veactor is added to the state vector.
  // Refer Lesson 8, Section 16 (UKF Augmentation).
  n_aug_ = n_x_ + 2;

  // Typical value for Lambda
  lambda_ = 3 - n_aug_;

  // Number of sigma points.
  // The  +1 is for the mean value.<F6><F6><F6>
  n_sig_ = 2 * n_aug_ + 1;

  cout << "n_x_ :" << n_x_ << "  n_aug_ :" << n_aug_ << "lambda_ :" << lambda_
<< "n_sig_ :" << n_sig_ << endl;
 
  // Predicted  Sigma points.
  Xsig_pred_ = MatrixXd::Zero( n_x_, n_sig_ );

  // Intialize the weights  
  weights_ = VectorXd::Zero( n_sig_ );
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for( int i = 1; i < n_sig_; i++ ) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  // Radar measurement noise covariance matrix initialization.
  R_radar_ = MatrixXd::Zero(3,3);

  R_radar_ << std_radr_*std_radr_, 0, 0,
           0, std_radphi_*std_radphi_, 0,
           0, 0, std_radrd_*std_radrd_;


 // Laser measurement noise covariance matrix initialization.
 R_lidar_ = MatrixXd::Zero(2,2);
 R_lidar_ << std_laspx_*std_laspx_, 0,
             0, std_laspy_*std_laspy_; 

 step_ = 0;
 
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  step_++;  

  if ( !is_initialized_ ) {
    
    // first measurement
    cout << "UKF::ProcessMeasurement() - First measurement type:" <<
             meas_package.sensor_type_ << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

      float rho = meas_package.raw_measurements_(0);
      float phi = meas_package.raw_measurements_(1);
      float rho_dot = meas_package.raw_measurements_(2);

      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rho_dot * cos(phi);
      float vy = rho_dot * sin(phi);
      float v = sqrt( vx*vx + vy*vy );
      x_ << px, py, v, 0, 0; 

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

      /**
     . Laser sensor measures only the position, not velocity.
      */

      x_ << meas_package.raw_measurements_[0],
            meas_package.raw_measurements_[1], 
            0, 0, 0;

      if ( fabs(x_(0)) < SMALL_NUMBER  and  fabs(x_(1)) < SMALL_NUMBER ) {

        x_(0) = SMALL_NUMBER;
        x_(1) = SMALL_NUMBER;

      }

    }

    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    cout << "UKF::ProcessMeasurement() - init done" << endl;
    return;
  }

  // Calculate time between measurements.
  double dt = ( meas_package.timestamp_ - time_us_ );

  // Convert it to seconds.
  dt = dt / 1000000.0;

  // Update time_us to use it for the next measurement.
  time_us_ = meas_package.timestamp_;

  // Predict
  Prediction( dt );

  if( meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);

  }
  
  if( meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {  
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Refer Lesson 8;, sections 16, 17, 18 - UKF Augmentation.

  cout << "UKF::Prediction(), delta_t: " << delta_t << endl;
  
  // Augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  
  // Augmented State Covariance Matrix
  MatrixXd P_aug = MatrixXd( n_aug_, n_aug_ );

  // Sigma point matrix
  MatrixXd Xsig_aug = MatrixXd( n_aug_, n_sig_ );  // n_sig_ = ((2 * n_aug_) + 1 )

  cout << "#1" << endl;

  // Create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  cout << "#2" << endl;

  // Create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;


  cout << "#3" << endl;

  // Create square root of matrix P
  MatrixXd L = P_aug.llt().matrixL();


  cout << "#4" << endl;

  // Create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  double sqrt_lambda_n_aug = sqrt( lambda_ + n_aug_ );
  VectorXd sqrt_lambda_n_aug_L;
  for( int i = 0; i < n_aug_; i++ ) {

    sqrt_lambda_n_aug_L = sqrt_lambda_n_aug * L.col(i);
    Xsig_aug.col(i+1) = x_aug + sqrt_lambda_n_aug_L;
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt_lambda_n_aug_L;

  }
 
  cout << "#5" << endl;

  // Predict the sigma points 
  // Refer: Lesson 8, Sections 19, 20, 21 - Sigma Point Prediction.
  for( int i = 0; i < n_sig_; i++ ) {
  
    // Extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // Avoid repeated computations.
    double sin_yaw = sin( yaw );
    double cos_yaw = cos( yaw );
    double yaw_dt = yaw + ( yawd * delta_t );

    // Predicted state values
    double px_p, py_p;

    // Avoid division by zero
    if ( fabs( yawd ) > SMALL_NUMBER) {

        double v_yawd = v/yawd;

        px_p = p_x + v_yawd * ( sin( yaw_dt ) - sin_yaw);
        py_p = p_y + v_yawd * ( cos_yaw - cos( yaw_dt ) );
    }
    else {
        double v_delta_t = v * delta_t;
        px_p = p_x + v_delta_t * cos_yaw;
        py_p = p_y + v_delta_t * sin_yaw;
    }


    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    double delta_t_sq = delta_t * delta_t;    

    // Add noise
    px_p = px_p + 0.5 * nu_a * delta_t_sq * cos_yaw;
    py_p = py_p + 0.5 * nu_a * delta_t_sq * sin_yaw;
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd *delta_t_sq;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // Write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
    
  } // end of for loop "Predict the sigma points"


  cout << "#6" << endl;
   
  // Refer Lesson 8. Sections 22,23,24: Predicted Mean and Covariance.

  // Predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  cout << "#7" << endl;
  // Predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    //while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    //while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    NormalizeAngle(&(x_diff(3)));

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

  cout << "#8" << endl;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  cout << "UKF::UpdateLidar(), sensor: "  << meas_package.sensor_type_ << endl;

  // Measurement dimension.
  int n_z = 2;

  // Create matrix for sigma points in measurement space.
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);
 
  // Call the UKFUpdate() function that is common to both Radar and Lidar
  // measurements.
  UKFUpdate( meas_package, Zsig, n_z);
    

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  cout << "UKF::UpdateRadar(), sensor: "  << meas_package.sensor_type_  << endl;

  // Refer Lesson 8, Sections 25, 26, 27: Measurement Prediction.

  // Set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero( n_z, n_sig_ );

  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) { 

    // Extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0,i) = sqrt( p_x*p_x + p_y*p_y );         // r
    Zsig(1,i) = atan2( p_y, p_x );                 // phi
    Zsig(2,i) = ( p_x*v1 + p_y*v2 ) / Zsig(0,i);   // r_dot

  }

  // Call the UKFUpdate() function that is common to both Radar and Lidar
  // measurements.
  UKFUpdate( meas_package, Zsig, n_z);


}

/**
 * Normalizes the angle to a value between -PI to PI.
 */
void UKF::NormalizeAngle( double *angle ) {

  while(*angle > M_PI) *angle -= 2. * M_PI;
  while(*angle < M_PI) *angle += 2. * M_PI; 


}


/**
 ** Updates the state and covariance matrix with the measurements.
 ** This function captures the updates that are applicable to both
 ** Radar and Lidar measurments.
  */
void UKF::UKFUpdate(MeasurementPackage meas_package, MatrixXd Zsig, int n_z) {

  // Refer Lesson 8, Sections 25, 26, 27 - Measurement Prediction

  //cout << "UKF::UKFUpdate(),  Zsig: " << Zsig << endl;
  cout << "UKFUpdate(), n_z: " << n_z << endl;
  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < n_sig_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }


  // Innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    //while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    //while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    NormalizeAngle(&(z_diff(1)));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }


  // Add measurement noise covariance matrix
  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  if( meas_package.sensor_type_ == MeasurementPackage::RADAR ) {
    R = R_radar_;
  }
  else if( meas_package.sensor_type_ == MeasurementPackage::LASER ) {
    R = R_lidar_;
  } 
 
  S = S + R;


  // Refer Lesson 8, Sections 28, 29, 30 - UKF Update. 
  
  // Create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  // Calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) { 

    // Residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Angle normalization
    if( meas_package.sensor_type_ == MeasurementPackage::RADAR ) {    
      NormalizeAngle(&(z_diff(1)));
    }

    // State difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Angle normalization
    NormalizeAngle(&(x_diff(3)));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();

  } // end of for loop.

  // Measurements
  VectorXd z = meas_package.raw_measurements_;

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // Residual
  VectorXd z_diff = z - z_pred;

  // Angle normalization
  if( meas_package.sensor_type_ == MeasurementPackage::RADAR ) {
    NormalizeAngle(&(z_diff(1)));
  }


  // Update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();


  // Calculate NIS

  if( meas_package.sensor_type_ == MeasurementPackage::RADAR ) {
    nis_radar_ = z_diff.transpose() * S.inverse() * z_diff;
    cout << "Step: " << step_ << "   nis_radar: " << nis_radar_ << endl;
    
  }
  else if( meas_package.sensor_type_ == MeasurementPackage::LASER ) {
    nis_laser_ = z_diff.transpose() * S.inverse() * z_diff;
    cout << "Step: " << step_ << "   nis_laser: " << nis_laser_ << endl;
  }

} // end of function UKF::UKFUpdate()
