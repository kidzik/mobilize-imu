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

  std::vector<std::string> m_datafiles;

  /************************************************************************/
  /* Number of observations used                                          */
  /************************************************************************/
  int m_num_sensors;

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
		std::vector<std::string> body_names,
		std::vector<std::string> datafiles
		)
    : m_model_path(model_path),
      m_imu_path(imu_path),
      m_ik_result_path(result_path),
      m_body_names(body_names),
      m_datafiles(datafiles)  {
    m_num_sensors = body_names.size();
  }

  ~OrientationIK()
  {

  }

  SimTK::Mat33 readData(std::ifstream* ifile) {
    double a;
    *ifile >> a;

    SimTK::Mat33 Rot;
    *ifile >> Rot[0][0];
    *ifile >> Rot[1][0];
    *ifile >> Rot[2][0];
    *ifile >> Rot[0][1];
    *ifile >> Rot[1][1];
    *ifile >> Rot[2][1];
    *ifile >> Rot[0][2];
    *ifile >> Rot[1][2];
    *ifile >> Rot[2][2];
    return Rot;
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

    // add markers
    addCustomMarkers(model, *markers, *imus);

    // result storage
    OpenSim::Kinematics kinematics(&model);
    kinematics.setRecordAccelerations(true);

    // Initial rotations with respect to opensim
    // right leg
    SimTK::Rotation rot_xsens_opensim(SimTK::BodyOrSpaceType::BodyRotationSequence,
				      -SimTK::Pi/2, SimTK::ZAxis,
				      0, SimTK::YAxis,
				      0, SimTK::XAxis);
    // left leg
    SimTK::Rotation rot_xsens_opensiml(SimTK::BodyOrSpaceType::BodyRotationSequence,
				       SimTK::Pi/2, SimTK::ZAxis,
				       SimTK::Pi, SimTK::YAxis,
				       0, SimTK::XAxis);

    model.getMultibodySystem().realize(state, SimTK::Stage::Report);

    std::vector<SimTK::Rotation> rotations_osim; // initial opensim rotations
    std::vector<SimTK::Rotation> rotations_initial; // initial rotation of observations
    std::vector<std::ifstream*> ifiles;

    for (int i=0; i < m_num_sensors; i++){
      const SimTK::Transform transform = model.getSimbodyEngine().getTransform(state, model.getBodySet().get(m_body_names[i].c_str()));
      rotations_osim.push_back(transform.R());
    }
    
    for (int i=0; i < m_num_sensors; i++)
      ifiles.push_back(new std::ifstream(m_datafiles[i]));

    // move to initial target
    ik.adoptAssemblyGoal(imus);

    // Get the first observation and set things up
    std::string line;
    for (int i=0; i < m_num_sensors; i++){
      for(int j=0; j < 6; j++){
	getline(*ifiles[i],line);
      }
      SimTK::Mat33 rot_matrix_obs = readData(ifiles[i]);
      rotations_initial.push_back(SimTK::Rotation(rot_matrix_obs).transpose());
    }

    std::vector<SimTK::Rotation> rotations_xsens_osim = rotations_initial;
    
    SimTK::Rotation reference_rot(SimTK::BodyOrSpaceType::BodyRotationSequence,
                                     -SimTK::Pi/2, SimTK::ZAxis,
                                     0, SimTK::YAxis,
                                     0, SimTK::XAxis);

    for (int i=0; i < m_num_sensors; i++){
      rotations_xsens_osim[i] = reference_rot * rotations_xsens_osim[i] * rotations_initial[1].transpose();
      std::cout << rotations_xsens_osim[i] << std::endl;
    }

    std::cout << rot_xsens_opensim << std::endl;
    std::cout << rot_xsens_opensiml << std::endl;
    for (int i=0; i < m_num_sensors; i++){
      imus->moveOneObservation(m_sensors_ox[i], rotations_xsens_osim[i] * rotations_initial[i] * rotations_initial[i].transpose() * rotations_xsens_osim[i].transpose() * rotations_osim[i]);
    }

    // setup inverse kinematics
    state.setTime(0.);
    ik.initialize(state);
    ik.assemble(state);
    kinematics.begin(state);

    viz.drawFrameNow(state);

    // Loop through all observations
    int iframe = 0;
    while(!(*ifiles[0]).eof())
      {
	iframe++;
	for (int i=0; i < m_num_sensors; i++){
	  SimTK::Mat33 rot_matrix_obs = readData(ifiles[i]);
	  imus->moveOneObservation(m_sensors_ox[i], rotations_xsens_osim[i] * rotations_initial[i] * SimTK::Rotation(rot_matrix_obs) * rotations_xsens_osim[i].transpose() * rotations_osim[i]);
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

    for (int i=0; i < m_num_sensors; i++)
      delete ifiles[i];
    ifiles.clear();

  }

private:

  /************************************************************************/
  /* Adds the markers for the line test problem                           */
  /************************************************************************/
  void addCustomMarkers(OpenSim::Model& model,
			SimTK::Markers& markers, SimTK::OrientationSensors& imus)
  {
    // Add sensors
    for(int i=0; i < m_num_sensors; i++){
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
    imus.defineObservationOrder(m_num_sensors, &names[0]);
    for(int i=0; i < m_num_sensors; i++){
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

    std::vector<std::string> bodies = {
      "femur_r",
      "pelvis",
      "tibia_r",
      "femur_l",
      "tibia_l"
    };
    std::vector<std::string> trackingFiles = {
      "dataimu6/MT_012005D6-009-000_00B4227C.txt",
      "dataimu6/MT_012005D6-009-000_00B4226B.txt",
      "dataimu6/MT_012005D6-009-000_00B4227D.txt",
      "dataimu6/MT_012005D6-009-000_00B421EE.txt",
      "dataimu6/MT_012005D6-009-000_00B421ED.txt"
    };

    OrientationIK ik(
		     "Full_LegPelvisModel.osim",
		     "futureOrientationInverseKinematics.trc",
		     "futureOrientationInverseKinematics",
		     bodies,
		     trackingFiles
		     );
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

  return 0;
}
