/* ------------------------------------------------------------------------- *
 *             OpenSim:  futureOrientationInverseKinematics.cpp               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Dimitar Stanev                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/OpenSim.h>
#include <Simbody.h>
#include <iostream>
#include <fstream>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Analyses/Kinematics.h>

/*
  Demonstrates how orientation sensors can be used for tracking the movements
  of the arm and to perform the inverse kinematics. Two IMUs are attached, one
  on the femur and the other on the tibia. The recordings are stored in a
  .trc file. The model is an extended version of the arm26. Currently, OpenSim
  does not provide direct tools for employing inverse kinematics based on the
  orientation sensors, while Simbody provides these facilities. This code
  uses the Simbody interface to execute the inverse kinematics for the specific
  problem and it is not a generic solution.
*/

#define DEBUG

char *convert(const std::string & s)
{
   char *pc = new char[s.size()+1];
   std::strcpy(pc, s.c_str());
   return pc; 
}

class OrientationIK
{
  /************************************************************************/
  /* Path variables                                                       */
  /************************************************************************/
  std::string m_model_path;
  std::string m_imu_path;
  std::string m_ik_result_path;

  /************************************************************************/
  /* Name of the bodies where the imus are attached                       */
  /************************************************************************/
  std::vector<std::string> m_body_names;

  /************************************************************************/
  /* Number of observations used                                          */
  /************************************************************************/
  static const int OSENSORS = 5;

  /************************************************************************/
  /* IMU indexes to point to the allocation space                         */
  /************************************************************************/
  std::vector<SimTK::OrientationSensors::OSensorIx> m_sensors_mx;
  std::vector<SimTK::OrientationSensors::ObservationIx> m_sensors_ox;
    


public:

  OrientationIK(
		std::string model_path,
		std::string imu_path,
		std::string result_path,
		std::vector<std::string> body_names
		)
    : m_model_path(model_path),
      m_imu_path(imu_path),
      m_ik_result_path(result_path),
      m_body_names(body_names)  {
  }

  ~OrientationIK()
  {

  }

  /************************************************************************/
  /* Runs the inverse kinematics                                          */
  /************************************************************************/
  void run()
  {
    // load model and data
    OpenSim::Model model(m_model_path);
    model.setUseVisualizer(true);

    // OpenSim::MarkerData imu_trc(m_imu_path);

    SimTK::State& state = model.initSystem();

    // setup
    SimTK::Assembler ik(model.updMultibodySystem());
    ik.setAccuracy(1e-5);
    SimTK::Markers* markers = new SimTK::Markers();
    SimTK::OrientationSensors* imus = new SimTK::OrientationSensors();

    SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();

    viz.setDesiredFrameRate(1000);
    viz.drawFrameNow(state);

    //getchar();

    // add markers
    addCustomMarkers(model, *markers, *imus);

    // result storage
    OpenSim::Kinematics kinematics(&model);
    kinematics.setRecordAccelerations(true);

    SimTK::Rotation rot_xsens_opensim(SimTK::BodyOrSpaceType::BodyRotationSequence,
				      -SimTK::Pi/2, SimTK::ZAxis,
				      0, SimTK::YAxis,
				      0, SimTK::XAxis);

    SimTK::Rotation rot_xsens_opensiml(SimTK::BodyOrSpaceType::BodyRotationSequence,
				       SimTK::Pi/2, SimTK::ZAxis,
				       SimTK::Pi, SimTK::YAxis,
				       0, SimTK::XAxis);

    model.getMultibodySystem().realize(state, SimTK::Stage::Report);
    std::vector<SimTK::Rotation> rotations_osim;

    for (int i=0; i<OSENSORS; i++){
      const SimTK::Transform transform = model.getSimbodyEngine().getTransform(state, model.getBodySet().get(m_body_names[i].c_str()));
      rotations_osim.push_back(transform.R());
    }
    
    std::vector<SimTK::Rotation> rotations_obs;
    std::vector<SimTK::Rotation> rotations_initial;
    std::vector<std::ifstream*> ifiles;

    std::ifstream ifile("dataimu6/MT_012005D6-009-000_00B4227C.txt"); // femur R
    ifiles.push_back(&ifile);
    std::ifstream ifileP("dataimu6/MT_012005D6-009-000_00B4226B.txt"); // pelvis
    ifiles.push_back(&ifileP);
    std::ifstream ifile2("dataimu6/MT_012005D6-009-000_00B4227D.txt"); // tibia
    ifiles.push_back(&ifile2);
    std::ifstream ifilel("dataimu6/MT_012005D6-009-000_00B421EE.txt"); // femur L
    ifiles.push_back(&ifilel);
    std::ifstream ifile2l("dataimu6/MT_012005D6-009-000_00B421ED.txt"); // tibia L
    ifiles.push_back(&ifile2l);

    std::vector<SimTK::Rotation> rotations_xsens_osim = {rot_xsens_opensim, rot_xsens_opensim, rot_xsens_opensim, rot_xsens_opensiml, rot_xsens_opensiml};

    std::string line;

    for (int i=0; i<OSENSORS; i++){
      for(int j=0; j < 6; j++){
	getline(*ifiles[i],line);
      }
      double a;
      *ifiles[i] >> a;

      SimTK::Mat33 Rot;
      *ifiles[i] >> Rot[0][0];
      *ifiles[i] >> Rot[1][0];
      *ifiles[i] >> Rot[2][0];
      *ifiles[i] >> Rot[0][1];
      *ifiles[i] >> Rot[1][1];
      *ifiles[i] >> Rot[2][1];
      *ifiles[i] >> Rot[0][2];
      *ifiles[i] >> Rot[1][2];
      *ifiles[i] >> Rot[2][2];

      *ifiles[i] >> a;
      rotations_obs.push_back(SimTK::Rotation(Rot));
    
      rotations_initial.push_back(SimTK::Rotation(Rot).transpose());
    }

    // move to initial target
    ik.adoptAssemblyGoal(imus);

    for (int i=0; i<OSENSORS; i++){
      imus->moveOneObservation(m_sensors_ox[i], rotations_xsens_osim[i] * rotations_initial[i] * rotations_obs[i] * rotations_xsens_osim[i].transpose() * rotations_osim[i]);
    }

    // setup inverse kinematics
    state.setTime(0.);
    ik.initialize(state);
    ik.assemble(state);
    kinematics.begin(state);

    viz.drawFrameNow(state);

    // getchar();

    int iframe = 0;
    while(!ifile.eof())
      {
	iframe++;
	for (int i=0; i<OSENSORS; i++){
	  double a;

	  SimTK::Mat33 Rot;
	  *ifiles[i] >> Rot[0][0];
	  *ifiles[i] >> Rot[1][0];
	  *ifiles[i] >> Rot[2][0];
	  *ifiles[i] >> Rot[0][1];
	  *ifiles[i] >> Rot[1][1];
	  *ifiles[i] >> Rot[2][1];
	  *ifiles[i] >> Rot[0][2];
	  *ifiles[i] >> Rot[1][2];
	  *ifiles[i] >> Rot[2][2];

	  *ifiles[i] >> a;
	  
	  imus->moveOneObservation(m_sensors_ox[i], rotations_xsens_osim[i] * rotations_initial[i] * SimTK::Rotation(Rot) * rot_xsens_opensim.transpose() * rotations_osim[i]);
	}

	// track
	state.updTime() = iframe * 1/1000;
	ik.track(state.getTime());
	ik.updateFromInternalState(state);

	viz.drawFrameNow(state);
	// report
#ifdef DEBUG
	std::cout << "Frame: " << iframe << " (t=" << state.getTime() << ")\n";
	std::cout << "Error: " << ik.calcCurrentErrorNorm() << "\n";
	std::flush(std::cout);
#endif

	// store
	kinematics.step(state, iframe);
      }

    kinematics.end(state);

    std::cout << "ik result path: " << m_ik_result_path << std::endl;

    // store results
    kinematics.printResults(m_ik_result_path, "");
  }

private:

  /************************************************************************/
  /* Adds the markers for the line test problem                           */
  /************************************************************************/
  void addCustomMarkers(OpenSim::Model& model,
			SimTK::Markers& markers, SimTK::OrientationSensors& imus)
  {
    // Add sensors
    for(int i=0; i < OSENSORS; i++){
      SimTK::OrientationSensors::OSensorIx m_mx = imus.addOSensor(m_body_names[i],
    			     model.updBodySet().get(m_body_names[i]).getMobilizedBodyIndex(),
    			     SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
    					     0, SimTK::ZAxis,
    					     0, SimTK::YAxis,
    					     0, SimTK::XAxis),
    			     1);
      m_sensors_mx.push_back(m_mx);
    }

    // Match observations with sensors
    std::vector<char*> names;
    std::transform(m_body_names.begin(), m_body_names.end(), std::back_inserter(names), convert);  
    imus.defineObservationOrder(OSENSORS, &names[0]);
    for(int i=0; i < OSENSORS; i++){
      m_sensors_ox.push_back(imus.getObservationIxForOSensor(m_sensors_mx[i]));
    }

  }
};


/**
 * Main function
 */
int main()
{
  try {

    OrientationIK ik(
		     "Full_LegPelvisModel.osim",
		     // "arm26.osim",
		     // "FullBody.osim",
		     "futureOrientationInverseKinematics.trc",
		     "futureOrientationInverseKinematics",
		     {"femur_r",
			 "pelvis",
			 "tibia_r",
			 "femur_l",
			 "tibia_l"
			 });
    ik.run();

  }
  catch (const std::exception& ex)
    {
      std::cout << "Exception: " << ex.what() << std::endl;
      return 1;
    }
  catch (...)
    {
      std::cout << "Unrecognized exception " << std::endl;
      return 1;
    }

  //system("pause");

  return 0;
}
