//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"
#include "Util.hpp"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDist_(0, 1) {

    /// create world
    world_ = std::make_unique<raisim::World>();

    /// add objects
    tocabi_ = world_->addArticulatedSystem(resourceDir_+"/tocabi/dyros_tocabi.urdf");
    tocabi_->setName("tocabi");
    tocabi_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    world_->addGround();

    /// get robot data
    gcDim_ = tocabi_->getGeneralizedCoordinateDim();
    gvDim_ = tocabi_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

    /// this is nominal configuration of tocabi
    gc_init_ << 0, 0, 0.92983, 1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
               0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
               0.0, 0.0, 0.0,
               0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
               0.0, 0.0,
               -0.3, -0.3, -1.5, 1.27, 1.0, 0.0, 1.0, 0.0;

    /// set pd gains
    jointPgain.resize(gvDim_); jointDgain.resize(gvDim_);
    jointPgain.setZero(); jointDgain.setZero(); 
    tocabi_->setPdGains(jointPgain, jointDgain);
    jointPgain.tail(nJoints_) << 2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                  2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                  6000.0, 10000.0, 10000.0,
                  400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0,
                  100.0, 100.0,
                  400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0;
                  
    jointDgain.tail(nJoints_) << 15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                  15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                  200.0, 100.0, 100.0,
                  10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0,
                  2.0, 2.0,
                  10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0;
    tocabi_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 34;
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    double action_std;
    READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
    actionStd_.setConstant(action_std);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    // footIndices_.insert(tocabi_->getBodyIdx("Pelvis_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_HipRoll_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_HipCenter_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Thigh_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Knee_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_HipRoll_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_HipCenter_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Thigh_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Knee_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("Waist1_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("Waist2_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("Upperbody_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Shoulder1_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Shoulder2_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Shoulder3_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Armlink_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Elbow_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Forearm_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Wrist1_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("R_Wrist2_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Shoulder1_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Shoulder2_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Shoulder3_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Armlink_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Elbow_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Forearm_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Wrist1_Link"));
    // footIndices_.insert(tocabi_->getBodyIdx("L_Wrist2_Link"));

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(tocabi_);
    }

    // Mocap
    mocapData_.resize(n_mocap_row, n_mocap_col);
    std::filesystem::path cwd = std::filesystem::current_path() / "motions/processed_data_tocabi_walk.txt";
    readTextFile(cwd.string(), mocapData_);
    target_data_qpos_.resize(nJoints_);
    target_data_qpos_ = gc_init_.tail(nJoints_);
  }

  void init() final { }

  void reset() final {
    tocabi_->setState(gc_init_, gv_init_);
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    // pTarget12_ = action.cast<double>();
    // pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    // pTarget12_ += actionMean_;
    // pTarget_.tail(nJoints_) = pTarget12_;


    // Mocap Debugging
    double local_time = std::fmod(time_, mocap_cycle_period_);
    double local_time_plus_init = fmod(local_time + init_mocap_data_idx_*mocap_cycle_dt_, mocap_cycle_period_);
    int mocap_data_idx = (init_mocap_data_idx_ + int(local_time / mocap_cycle_dt_)) % (n_mocap_row-1);
    int next_idx = mocap_data_idx + 1;

    for (int i=0; i<nJoints_; i++){
      target_data_qpos_(i) = cubic(local_time_plus_init, mocapData_(mocap_data_idx,0), mocapData_(next_idx,0), mocapData_(mocap_data_idx,1+i), mocapData_(next_idx,1+i), 0.0, 0.0);
    }
    
    // pTarget_.tail(nJoints_) = target_data_qpos_;
    // tocabi_->setPdTarget(pTarget_, vTarget_);


    // Eigen::VectorXd gc_init_tmp; gc_init_tmp.resize(gcDim_); gc_init_tmp.setZero();
    // gc_init_tmp = gc_init_;
    // gc_init_tmp.tail(nJoints_) = target_data_qpos_;
    // tocabi_->setState(gc_init_tmp, gv_init_);

    Eigen::VectorXd tau;
    tau.resize(gvDim_); tau.setZero();
    
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
    tocabi_->getState(gc_, gv_);
    tau.tail(nJoints_) = jointPgain.tail(nJoints_).cwiseProduct(target_data_qpos_-gc_.tail(nJoints_)) - jointDgain.tail(nJoints_).cwiseProduct(gv_.tail(nJoints_));
    tocabi_->setGeneralizedForce(tau);

      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }
    time_ += simulation_dt_;

    // Obervation
    updateObservation();

    // Reward
    // rewards_.record("torque", tocabi_->getGeneralizedForce().squaredNorm());
    // rewards_.record("forwardVel", std::min(4.0, bodyLinearVel_[0]));

    return rewards_.sum();
  }

  void updateObservation() {
    tocabi_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    obDouble_ << gc_[2], /// body height
        rot.e().row(2).transpose(), /// body orientation
        gc_.segment(7,12), /// joint angles
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gv_.segment(6,12); /// joint velocity
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    // for(auto& contact: tocabi_->getContacts())
    //   if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
    //     return true;

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };


 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* tocabi_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_;
  double terminalRewardCoeff_ = -10.;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;

  Eigen::VectorXd jointPgain, jointDgain;

  double time_ = 0.0;

  const static int n_mocap_row = 3600;
  const static int n_mocap_col = 34;
  Eigen::MatrixXd mocapData_;
  int mocap_data_idx_ = 0;
  int init_mocap_data_idx_ = 0;
  double mocap_cycle_dt_ = 0.0005;
  double mocap_cycle_period_ = (n_mocap_row-1)*mocap_cycle_dt_;
  Eigen::VectorXd target_data_qpos_;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  std::normal_distribution<double> normDist_;
  thread_local static std::mt19937 gen_;
};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;

}

