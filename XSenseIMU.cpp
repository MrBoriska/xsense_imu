#include "XSenseIMU.hpp"


void XSenseIMU::start() {
    //tickPeriodically();


    cout << "Creating XsControl object..." << endl;
	this->control = XsControl::construct();
	assert(control != 0);

	// Lambda function for error handling
	auto handleError = [=](string errorString)
	{
		control->destruct();
		cout << errorString << endl;
		cout << "Press [ENTER] to continue." << endl;
		cin.get();
		return -1;
	};

	cout << "Scanning for devices..." << endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			this->mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
		return handleError("No MTi device found. Aborting.");

	cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << endl;

	cout << "Opening port..." << endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.");

	// Get the device object
	this->device = control->device(mtPort.deviceId());
	assert(device != 0);

	cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << endl;

	// Create and attach callback handler to device
	XSenseCallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	cout << "Putting device into configuration mode..." << endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.");

	cout << "Configuring the device..." << endl;

	// Important for Public XDA!
	// Call this function if you want to record a mtb file:
	device->readEmtsAndDeviceConfiguration();

	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));

	if (device->deviceId().isImu())
	{
		configArray.push_back(XsOutputConfiguration(XDI_DeltaV, 0));
		configArray.push_back(XsOutputConfiguration(XDI_DeltaQ, 0));
		configArray.push_back(XsOutputConfiguration(XDI_MagneticField, 0));
	}
	else if (device->deviceId().isVru() || device->deviceId().isAhrs())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
	}
	else if (device->deviceId().isGnss())
	{
		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));
		configArray.push_back(XsOutputConfiguration(XDI_LatLon, 0));
		configArray.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, 0));
		configArray.push_back(XsOutputConfiguration(XDI_VelocityXYZ, 0));
	}
	else
	{
		return handleError("Unknown device while configuring. Aborting.");
	}

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.");

	/*
    cout << "Creating a log file..." << endl;
	string logFileName = "logfile.mtb";
	if (device->createLogFile(logFileName) != XRV_OK)
		return handleError("Failed to create a log file. Aborting.");
	else
		cout << "Created a log file: " << logFileName.c_str() << endl;
    */

	cout << "Putting device into measurement mode..." << endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.");

	cout << "Starting recording..." << endl;
	if (!device->startRecording())
		return handleError("Failed to start recording. Aborting.");


    if (callback.packetAvailable()) {
        cout << setw(5) << fixed << setprecision(2);

        // Retrieve a packet
        XsDataPacket packet = callback.getNextPacket();
        if (packet.containsCalibratedData())
        {
            XsVector acc = packet.calibratedAcceleration();
            cout << "\r"
                << "Acc X:" << acc[0]
                << ", Acc Y:" << acc[1]
                << ", Acc Z:" << acc[2];

            XsVector gyr = packet.calibratedGyroscopeData();
            cout << " |Gyr X:" << gyr[0]
                << ", Gyr Y:" << gyr[1]
                << ", Gyr Z:" << gyr[2];

            XsVector mag = packet.calibratedMagneticField();
            cout << " |Mag X:" << mag[0]
                << ", Mag Y:" << mag[1]
                << ", Mag Z:" << mag[2];
        }

        if (packet.containsOrientation())
        {
            XsQuaternion quaternion = packet.orientationQuaternion();
            cout << "\r"
                << "q0:" << quaternion.w()
                << ", q1:" << quaternion.x()
                << ", q2:" << quaternion.y()
                << ", q3:" << quaternion.z();

            XsEuler euler = packet.orientationEuler();
            cout << " |Roll:" << euler.roll()
                << ", Pitch:" << euler.pitch()
                << ", Yaw:" << euler.yaw();
        }

        if (packet.containsLatitudeLongitude())
        {
            XsVector latLon = packet.latitudeLongitude();
            cout << " |Lat:" << latLon[0]
                << ", Lon:" << latLon[1];
        }

        if (packet.containsAltitude())
            cout << " |Alt:" << packet.altitude();

        if (packet.containsVelocity())
        {
            XsVector vel = packet.velocity(XDI_CoordSysEnu);
            cout << " |E:" << vel[0]
                << ", N:" << vel[1]
                << ", U:" << vel[2];
        }
        
        cout << flush;
    }

}
void XSenseIMU::tick() {
    LOG_INFO(get_message().c_str());


    auto imu_proto = tx_imu().initProto();
    imu_proto.setLinearAccelerationX(0);
    imu_proto.setLinearAccelerationY(0);
    imu_proto.setLinearAccelerationZ(0);
    imu_proto.setAngularVelocityX(0);
    imu_proto.setAngularVelocityY(0);
    imu_proto.setAngularVelocityZ(0);
    imu_proto.setAnglePitch(0);
    imu_proto.setAngleRoll(0);
    imu_proto.setAngleYaw(0);
    tx_imu().publish();


}
void XSenseIMU::stop() {
    cout << "Stopping recording..." << endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.");

    /*
	cout << "Closing log file..." << endl;
	if (!device->closeLogFile())
		return handleError("Failed to close log file. Aborting.");
    */

	cout << "Closing port..." << endl;
	control->closePort(mtPort.portName().toStdString());

	cout << "Freeing XsControl object..." << endl;
	control->destruct();

	cout << "Successful exit." << endl;

}