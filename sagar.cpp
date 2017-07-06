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
    std::string m_femur_body_name;
    std::string m_tibia_body_name;

    std::string m_femurl_body_name;
    std::string m_tibial_body_name;

    std::string m_pelvis_body_name;

    /************************************************************************/
    /* Number of observations used                                          */
    /************************************************************************/
    static const int OSENSORS = 5;

    /************************************************************************/
    /* IMU indexes to point to the allocation space                         */
    /************************************************************************/
    SimTK::OrientationSensors::OSensorIx m_femur_mx;
    SimTK::OrientationSensors::ObservationIx m_femur_ox;
    SimTK::OrientationSensors::OSensorIx m_tibia_mx;
    SimTK::OrientationSensors::ObservationIx m_tibia_ox;
    SimTK::OrientationSensors::OSensorIx m_femurl_mx;
    SimTK::OrientationSensors::ObservationIx m_femurl_ox;
    SimTK::OrientationSensors::OSensorIx m_tibial_mx;
    SimTK::OrientationSensors::ObservationIx m_tibial_ox;
    
   SimTK::OrientationSensors::OSensorIx m_pelvis_mx;
    SimTK::OrientationSensors::ObservationIx m_pelvis_ox;


public:

    OrientationIK(
        std::string model_path,
        std::string imu_path,
        std::string result_path,
        std::string femur_body_name, std::string tibia_body_name,
        std::string femurl_body_name, std::string tibial_body_name,
	std::string pelvis_body_name)
    : m_model_path(model_path),
    m_imu_path(imu_path),
    m_ik_result_path(result_path),
    m_femur_body_name(femur_body_name), m_tibia_body_name(tibia_body_name),
      m_femurl_body_name(femurl_body_name), m_tibial_body_name(tibial_body_name),
       m_pelvis_body_name(pelvis_body_name)
    {

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
	const SimTK::Transform transform = model.getSimbodyEngine().getTransform(state, model.getBodySet().get(m_femur_body_name.c_str()));
	const SimTK::Rotation rotation_femur = transform.R();
	const SimTK::Transform transform_tibia = model.getSimbodyEngine().getTransform(state, model.getBodySet().get(m_tibia_body_name.c_str()));
	const SimTK::Rotation rotation_tibia = transform_tibia.R();

 	 const SimTK::Transform transforml = model.getSimbodyEngine().getTransform(state, model.getBodySet().get(m_femurl_body_name.c_str()));
  	const SimTK::Rotation rotation_femurl = transforml.R();
  	const SimTK::Transform transform_tibial = model.getSimbodyEngine().getTransform(state, model.getBodySet().get(m_tibial_body_name.c_str()));
  	const SimTK::Rotation rotation_tibial = transform_tibial.R();
	
	const SimTK::Transform transform_pelvis = model.getSimbodyEngine().getTransform(state, model.getBodySet().get(m_pelvis_body_name.c_str()));
	const SimTK::Rotation rotation_pelvis = transform.R();

	SimTK::Rotation rot, rot2, rotl, rot2l, rotPelvis;
	SimTK::Mat33 Rot, Rot2, Rotl, Rot2l, RotPelvis;

       // std::ifstream ifile("../dataimu/MT_012005D6-001-000_00B4226C.txt");
	// std::ifstream ifile("../dataimu/2imusallmotions-000_00B4226C.txt");
	// std::ifstream ifile("../dataimu/justshoulderallmotions-000_00B4226C.txt");
  //  std::ifstream ifile("../dataimu2/differentpose2-000_00B4227D.txt");
  // std::ifstream ifile("../dataimu2/differentpose-000_00B4227D.txt");
  // std::ifstream ifile("../dataimu2/moving2planes-000_00B4227D.txt");
  // std::ifstream ifile("../dataimu2/movingX4fullrotation-000_00B4227D.txt");
  // std::ifstream ifile("../dataimu2/movingZ-000_00B4227D.txt");

// std::ifstream ifile("../dataimu3/walking-000_00B4227C.txt");
// std::ifstream ifile("../dataimu3/rightleg-000_00B4227C.txt");
// std::ifstream ifile("../dataimu3/rightleg2-000_00B4227C.txt");
// std::ifstream ifile("../dataimu3/rightarm1-000_00B42263.txt");
// std::ifstream ifile("../dataimu3/rightarmfast-000_00B42263.txt");


// std::ifstream ifile("../dataimu4/rightarm90_1-000_00B42263.txt");
// std::ifstream ifile("../dataimu5/rleg1-000_00B4227D.txt");
// std::ifstream ifile("../dataimu5/rightleg2 (2)-000_00B4227D.txt");


// std::ifstream ifile("../dataimu5/walking (2)-000_00B4227D.txt");

//std::ifstream ifile("../dataimu6/MT_012005D6-001-000_00B4227C.txt");
 std::ifstream ifile("../dataimu6/MT_012005D6-009-000_00B4227C.txt");
// std::ifstream ifile("../dataimu6/MT_012005D6-003-000_00B4227C.txt");


	std::string line;
	for(int i=0; i < 6; i++){
		getline(ifile,line);
	}

	double a, aP;
	ifile >> a;

	ifile >> Rot[0][0];
	ifile >> Rot[1][0];
	ifile >> Rot[2][0];
	ifile >> Rot[0][1];
	ifile >> Rot[1][1];
	ifile >> Rot[2][1];
	ifile >> Rot[0][2];
	ifile >> Rot[1][2];
	ifile >> Rot[2][2];

	ifile >> a;
	rot = SimTK::Rotation(Rot);

  // std::cout << "rot: " << rot << std::endl;

	SimTK::Rotation R_femur =  rot.transpose();
	//SimTK::Rotation R_femur =  rot.transpose()*rotation_femur;

// std::ifstream ifile2("../dataimu3/walking-000_00B4227D.txt");
// std::ifstream ifile2("../dataimu3/rightleg-000_00B4227D.txt");
// std::ifstream ifile2("../dataimu3/rightleg2-000_00B4227D.txt");
// std::ifstream ifile2("../dataimu3/rightarm1-000_00B4227B.txt");
// std::ifstream ifile2("../dataimu3/rightarmfast-000_00B4227B.txt");

// std::ifstream ifile2("../dataimu4/rightarm90_1-000_00B4227B.txt");
// std::ifstream ifile2("../dataimu5/rleg1-000_00B4227D.txt");
// std::ifstream ifile2("../dataimu5/rleg1-000_00B4227C.txt");
// std::ifstream ifile2("../dataimu5/rightleg2 (2)-000_00B4227C.txt");
//std::ifstream ifile2("../dataimu6/MT_012005D6-001-000_00B4227D.txt");
 std::ifstream ifile2("../dataimu6/MT_012005D6-009-000_00B4227D.txt");
// std::ifstream ifile2("../dataimu6/MT_012005D6-003-000_00B4227D.txt");


	for(int i=0; i < 6; i++){
		getline(ifile2,line);
	}

	double a2;
	ifile2 >> a2;

	ifile2 >> Rot2[0][0];
	ifile2 >> Rot2[1][0];
	ifile2 >> Rot2[2][0];
	ifile2 >> Rot2[0][1];
	ifile2 >> Rot2[1][1];
	ifile2 >> Rot2[2][1];
	ifile2 >> Rot2[0][2];
	ifile2 >> Rot2[1][2];
	ifile2 >> Rot2[2][2];

	ifile2 >> a2;
	rot2 = SimTK::Rotation(Rot2);

	SimTK::Rotation R_tibia =  rot2.transpose();


  //std::ifstream ifilel("../dataimu6/MT_012005D6-001-000_00B421EE.txt");
   std::ifstream ifilel("../dataimu6/MT_012005D6-009-000_00B421EE.txt");
  // std::ifstream ifilel("../dataimu6/MT_012005D6-003-000_00B421EE.txt");


  	for(int i=0; i < 6; i++){
  		getline(ifilel,line);
  	}

  	ifilel >> a;

  	ifilel >> Rotl[0][0];
  	ifilel >> Rotl[1][0];
  	ifilel >> Rotl[2][0];
  	ifilel >> Rotl[0][1];
  	ifilel >> Rotl[1][1];
  	ifilel >> Rotl[2][1];
  	ifilel >> Rotl[0][2];
  	ifilel >> Rotl[1][2];
  	ifilel >> Rotl[2][2];

  	ifilel >> a;
  	rotl = SimTK::Rotation(Rotl);

    // std::cout << "rot: " << rot << std::endl;

  	SimTK::Rotation R_femurl =  rotl.transpose();
  	//SimTK::Rotation R_femur =  rot.transpose()*rotation_femur;

  // std::ifstream ifile2("../dataimu3/walking-000_00B4227D.txt");
  // std::ifstream ifile2("../dataimu3/rightleg-000_00B4227D.txt");
  // std::ifstream ifile2("../dataimu3/rightleg2-000_00B4227D.txt");
  // std::ifstream ifile2("../dataimu3/rightarm1-000_00B4227B.txt");
  // std::ifstream ifile2("../dataimu3/rightarmfast-000_00B4227B.txt");

  // std::ifstream ifile2("../dataimu4/rightarm90_1-000_00B4227B.txt");
  // std::ifstream ifile2("../dataimu5/rleg1-000_00B4227D.txt");
  // std::ifstream ifile2("../dataimu5/rleg1-000_00B4227C.txt");
  // std::ifstream ifile2("../dataimu5/rightleg2 (2)-000_00B4227C.txt");
 // std::ifstream ifile2l("../dataimu6/MT_012005D6-001-000_00B421ED.txt");
   std::ifstream ifile2l("../dataimu6/MT_012005D6-009-000_00B421ED.txt");
  // std::ifstream ifile2l("../dataimu6/MT_012005D6-003-000_00B421ED.txt");


  	for(int i=0; i < 6; i++){
  		getline(ifile2l,line);
  	}

  	ifile2l >> a2;

  	ifile2l >> Rot2l[0][0];
  	ifile2l >> Rot2l[1][0];
  	ifile2l >> Rot2l[2][0];
  	ifile2l >> Rot2l[0][1];
  	ifile2l >> Rot2l[1][1];
  	ifile2l >> Rot2l[2][1];
  	ifile2l >> Rot2l[0][2];
  	ifile2l >> Rot2l[1][2];
  	ifile2l >> Rot2l[2][2];

  	ifile2l >> a2;
  	rot2l = SimTK::Rotation(Rot2l);

  	SimTK::Rotation R_tibial =  rot2l.transpose();
   
	std::ifstream ifileP("../dataimu6/MT_012005D6-009-000_00B4226B.txt");

  	for(int i=0; i < 6; i++){
  		getline(ifileP,line);
  	}

  	ifileP >> aP;

  	ifileP >> RotPelvis[0][0];
  	ifileP >> RotPelvis[1][0];
  	ifileP >> RotPelvis[2][0];
  	ifileP >> RotPelvis[0][1];
  	ifileP >> RotPelvis[1][1];
  	ifileP >> RotPelvis[2][1];
  	ifileP >> RotPelvis[0][2];
  	ifileP >> RotPelvis[1][2];
  	ifileP >> RotPelvis[2][2];
  	
	ifileP >> aP;
  	rotPelvis = SimTK::Rotation(RotPelvis);

  	SimTK::Rotation R_pelvis =  rotPelvis.transpose();

        // move to initial target
        ik.adoptAssemblyGoal(imus);

        imus->moveOneObservation(m_femur_ox,   rot_xsens_opensim * R_femur * rot * rot_xsens_opensim.transpose() * rotation_femur);
        imus->moveOneObservation(m_tibia_ox,  rot_xsens_opensim * R_tibia * rot2 * rot_xsens_opensim.transpose() * rotation_tibia);

        imus->moveOneObservation(m_femurl_ox,   rot_xsens_opensiml * R_femurl * rotl * rot_xsens_opensiml.transpose() * rotation_femurl);
        imus->moveOneObservation(m_tibial_ox,  rot_xsens_opensiml * R_tibial * rot2l * rot_xsens_opensiml.transpose() * rotation_tibial);
        
	imus->moveOneObservation(m_pelvis_ox,  rot_xsens_opensim * R_pelvis * rotPelvis * rot_xsens_opensim.transpose() * rotation_pelvis);

	// setup inverse kinematics
        state.setTime(0.);
        ik.initialize(state);
        ik.assemble(state);
        kinematics.begin(state);

        viz.drawFrameNow(state);

        // getchar();

	int i = 0;
        while(!ifile.eof())
	{
		i++;
		ifile >> Rot[0][0];
		ifile >> Rot[1][0];
		ifile >> Rot[2][0];
		ifile >> Rot[0][1];
		ifile >> Rot[1][1];
		ifile >> Rot[2][1];
		ifile >> Rot[0][2];
		ifile >> Rot[1][2];
		ifile >> Rot[2][2];

		ifile >> a;
		rot = SimTK::Rotation(Rot);

		ifile2 >> Rot2[0][0];
		ifile2 >> Rot2[1][0];
		ifile2 >> Rot2[2][0];
		ifile2 >> Rot2[0][1];
		ifile2 >> Rot2[1][1];
		ifile2 >> Rot2[2][1];
		ifile2 >> Rot2[0][2];
		ifile2 >> Rot2[1][2];
		ifile2 >> Rot2[2][2];

		ifile2 >> a2;
		rot2 = SimTK::Rotation(Rot2);

    		ifilel >> Rotl[0][0];
    		ifilel >> Rotl[1][0];
    		ifilel >> Rotl[2][0];
    		ifilel >> Rotl[0][1];
    		ifilel >> Rotl[1][1];
    		ifilel >> Rotl[2][1];
    		ifilel >> Rotl[0][2];
    		ifilel >> Rotl[1][2];
    		ifilel >> Rotl[2][2];

    		ifilel >> a;
    		rotl = SimTK::Rotation(Rotl);

    		ifile2l >> Rot2l[0][0];
  		ifile2l >> Rot2l[1][0];
  		ifile2l >> Rot2l[2][0];
  		ifile2l >> Rot2l[0][1];
  		ifile2l >> Rot2l[1][1];
  		ifile2l >> Rot2l[2][1];
  		ifile2l >> Rot2l[0][2];
  		ifile2l >> Rot2l[1][2];
  		ifile2l >> Rot2l[2][2];

  		ifile2l >> a2;
  		rot2l = SimTK::Rotation(Rot2l);
    		
    		rotPelvis = SimTK::Rotation(RotPelvis);

    		ifileP >> RotPelvis[0][0];
  		ifileP >> RotPelvis[1][0];
  		ifileP >> RotPelvis[2][0];
  		ifileP >> RotPelvis[0][1];
  		ifileP >> RotPelvis[1][1];
  		ifileP >> RotPelvis[2][1];
  		ifileP >> RotPelvis[0][2];
  		ifileP >> RotPelvis[1][2];
  		ifileP >> RotPelvis[2][2];

  		ifileP >> aP;
  		rotPelvis = SimTK::Rotation(RotPelvis);

  		imus->moveOneObservation(m_pelvis_ox,   rot_xsens_opensim * R_pelvis * rotPelvis * rot_xsens_opensim.transpose() * rotation_pelvis);

  		imus->moveOneObservation(m_femur_ox,   rot_xsens_opensim * R_femur * rot * rot_xsens_opensim.transpose() * rotation_femur);
  		imus->moveOneObservation(m_tibia_ox,  rot_xsens_opensim * R_tibia * rot2 * rot_xsens_opensim.transpose() * rotation_tibia);

    		imus->moveOneObservation(m_femurl_ox,   rot_xsens_opensiml * R_femurl * rotl * rot_xsens_opensiml.transpose() * rotation_femurl);
    		imus->moveOneObservation(m_tibial_ox,  rot_xsens_opensiml * R_tibial * rot2l * rot_xsens_opensiml.transpose() * rotation_tibial);

		// track
            	state.updTime() = i * 1/1000;
            	ik.track(state.getTime());
            	ik.updateFromInternalState(state);

              viz.drawFrameNow(state);
            // report
    #ifdef DEBUG
            std::cout << "Frame: " << i << " (t=" << state.getTime() << ")\n";
            std::cout << "Error: " << ik.calcCurrentErrorNorm() << "\n";
            std::flush(std::cout);
    #endif

            // store
            kinematics.step(state, i);
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
	// add orientation osensor
        /*
        m_femur_mx = imus.addOSensor(
            "femur_r",
            model.updBodySet().get(m_femur_body_name),
            SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
            0, SimTK::ZAxis,
          0, SimTK::YAxis,
        0, SimTK::XAxis),
            1);

        m_tibia_mx = imus.addOSensor(
            "tibia_r",
            model.updBodySet().get(m_tibia_body_name),
            SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
            0, SimTK::ZAxis,
            0, SimTK::YAxis,
          0, SimTK::XAxis),
            1);
            */

            m_femur_mx = imus.addOSensor(
                "femur_r",
                model.updBodySet().get(m_femur_body_name).getMobilizedBodyIndex(),
                SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
                0, SimTK::ZAxis,
              0, SimTK::YAxis,
            0, SimTK::XAxis),
                1);
            
	    m_pelvis_mx = imus.addOSensor(
                "pelvis",
                model.updBodySet().get(m_pelvis_body_name).getMobilizedBodyIndex(),
                SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
                0, SimTK::ZAxis,
              0, SimTK::YAxis,
            0, SimTK::XAxis),
                1);

            m_tibia_mx = imus.addOSensor(
                "tibia_r",
                model.updBodySet().get(m_tibia_body_name).getMobilizedBodyIndex(),
                SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
                0, SimTK::ZAxis,
                0, SimTK::YAxis,
              0, SimTK::XAxis),
                1);

            m_femurl_mx = imus.addOSensor(
                "femur_l",
                model.updBodySet().get(m_femurl_body_name).getMobilizedBodyIndex(),
                SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
                0, SimTK::ZAxis,
              0, SimTK::YAxis,
            0, SimTK::XAxis),
                1);

            m_tibial_mx = imus.addOSensor(
                "tibia_l",
                model.updBodySet().get(m_tibial_body_name).getMobilizedBodyIndex(),
                SimTK::Rotation(SimTK::BodyOrSpaceType::BodyRotationSequence,
                0, SimTK::ZAxis,
                0, SimTK::YAxis,
              0, SimTK::XAxis),
                1);


        // finalize observation order (to allocate ObservationIx)

	static const char* osensor_observation_order[OSENSORS] = {"femur_r", "tibia_r", "femur_l", "tibia_l", "pelvis"};
        imus.defineObservationOrder(OSENSORS, osensor_observation_order);

        // get all ObservationIx
        m_femur_ox = imus.getObservationIxForOSensor(m_femur_mx);
        m_tibia_ox = imus.getObservationIxForOSensor(m_tibia_mx);

        m_femurl_ox = imus.getObservationIxForOSensor(m_femurl_mx);
        m_tibial_ox = imus.getObservationIxForOSensor(m_tibial_mx);

        m_pelvis_ox = imus.getObservationIxForOSensor(m_pelvis_mx);

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
            "femur_r",
            "tibia_r",
            "femur_l",
            "tibia_l",
	     "pelvis");
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
