#include <xsensdeviceapi.h> // The Xsens device API header
#include "serialkey.h"
#include <iostream>
#include <list>
#include <iomanip>
#include <stdexcept>
#include <xsens/xstime.h>
#include <xsens/xsmutex.h>
#include <conio.h> // for non ANSI _kbhit() and _getch()

//----------------------------------------------------------
#include "Urg_driver.h"
#include "Connection_information.h"
#include <iostream>
#include <string>
#include <windows.h>
#include <process.h>    
#include <stddef.h>
#include <stdlib.h>
#include <conio.h>
#include <time.h>
#include <sstream>
#include <fstream>

//-------------------------------------------------------------
#include <iostream>  
#include <boost/date_time/local_time/local_time.hpp>  
#include <ctime>//boost library for time();

//--------------------------------------------------------------
using namespace boost::gregorian;  
using namespace boost::local_time;  
using namespace boost::posix_time;  


using namespace qrk;
using namespace std;

int const Angel = 78;
char repeat = 'c';   
LONGLONG timebegin;
Urg_driver urg;

const boost::posix_time::ptime epoch(boost::gregorian::date(1970, boost::gregorian::Jan, 1));

inline static int64_t GetCurrentStamp64()
{
	boost::posix_time::time_duration time_from_epoch =
		boost::posix_time::microsec_clock::universal_time() - epoch;
	//boost::posix_time::second_clock::universal_time() - epoch;
	return time_from_epoch.total_milliseconds();
	//return  time_from_epoch.total_microseconds();
}

//----------------------------------------------------------

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5) :
		m_maxNumberOfPacketsInBuffer(maxBufferSize),
		m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		XsMutexLocker lock(m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		XsMutexLocker lock(m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	virtual void onDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		XsMutexLocker lock(m_mutex);
		assert(packet != 0);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
		{
			(void)getNextPacket();
		}
		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable XsMutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	std::list<XsDataPacket> m_packetBuffer;
};

inline std::string int2string(int a)
{
	std::stringstream strm;
	std::string str;
	strm << a ;
	strm >> str;
	return str;
}

//CallbackHandler * callback;
void Function(void *cback)
{
	CallbackHandler * callback = (CallbackHandler*)cback; 
	//while (!_kbhit())
//	callback = (CallbackHandler*)cback;
	time_t now; 
	now =time(NULL);

	ofstream out("E:/binding/" + int2string(now) + "-Xsnes.txt");

	while( repeat != 'c' && repeat != 'q' )
  {
		//if (callback.packetAvailable())
	  if (callback->packetAvailable())
		{
			// Retrieve a packet
			/////XsDataPacket packet = callback.getNextPacket();
			XsDataPacket packet = callback->getNextPacket();
			// Get the quaternion data
			XsQuaternion quaternion = packet.orientationQuaternion();
			XsEuler euler = packet.orientationEuler();
			XsVector speed = packet.velocity();
			//std::cout << GetCurrentStamp64()<<"  "//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			 //std::cout << time(NULL)<<"  "		
			//std::cout <<(boost::posix_time::microsec_clock::universal_time() - epoch).total_milliseconds()<<"  "
			std::cout << packet.timeOfArrival().msTime()<<"  "
				<< " q0:" << quaternion.m_w
				<< ",q1:" << quaternion.m_x
				<< ",q2:" << quaternion.m_y
				<< ",q3:" << quaternion.m_z <<endl;
		/*	std::cout << "V-X"  << std::setw(7) << std::fixed << std::setprecision(2) << speed[0]
			       	 << ",V-Y:" << std::setw(7) << std::fixed << std::setprecision(2) << speed[1]
				     << ",V-Z:"   << std::setw(7) << std::fixed << std::setprecision(2) << speed[2]
				     << " " <<std::endl;*/
		 //   out  <<XsTime::timeStampNow()<<":"  /////////////////NEED TO REPLACE!
		 //   out  << GetCurrentStamp64()<<":"  //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		//  out  <<(boost::posix_time::microsec_clock::universal_time() - epoch).total_milliseconds()<<":"
			out <<packet.timeOfArrival().msTime()<<":"
			    << quaternion.m_w <<":"<< quaternion.m_x<<":" << quaternion.m_y<<":" << quaternion.m_z<<":" 
				<< speed[0]<<":" << speed[1]<<":" << speed[2] <<endl;
			out.flush();
		}
		//XsTime::msleep(0);
	}
	out.close();
	cout <<"Xsens complate once!"<<endl;
	//	_getch();
	::_endthread();
	
}

void print_data(const Urg_driver& urg,
		      const vector<long>& data,
			  const vector<unsigned short>& intensity,
			  long time_stamp, 
			  ofstream &out,
			  LONGLONG mybegin)
{
#if 0
		int front_index = urg.step2index(0); 
		cout << data[front_index] << " [mm], "
			<< intensity[front_index] << " [1], ("
			<< time_stamp << " [msec])" << endl;

		out << data[front_index] << " [mm], "
			<< intensity[front_index] << " [1], ("
			<< time_stamp << " [msec])" << endl;
#else
          
		size_t data_n = data.size();
	//	long long time = XsTime::timeStampNow();/////////////////REPLACE			
	//   long long time = GetCurrentStamp64();//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    //   long long time = (boost::posix_time::microsec_clock::universal_time() - epoch).total_milliseconds();
	
		out << data_n <<" x86isnice"<< endl;
		for (size_t i = 0; i < data_n; ++i)
		{
			//  cout << i << ", " << data[i] << ", " << intensity[i] << endl;		
			double radian = urg.index2rad(i);
			long x = static_cast<long>(data[i] * sin(radian));
			long y = static_cast<long>(data[i] * cos(radian));
			//  cout << "(" << x << ", " << y << ")"  << time_stamp << endl;
			//out << x << ":" << y <<":"<< time_stamp <<":"<<intensity[i]<<":"<< endl;
			//out << x << ":" << y <<":"<< time <<":"<<intensity[i]<<":"<< endl;
			out << x << ":" << y <<":"<< mybegin+time_stamp <<":"<<intensity[i]<<":"<< endl;
		}
		//cout << "# n = " << data_n << ", timestamp = " << time_stamp << endl;
		//cout << "# n = " << data_n << ", timestamp = " << time << endl;
	   cout << "# n = " << data_n << ", timestamp = " << mybegin+time_stamp << " "<<GetCurrentStamp64()<< endl;
#endif
}

void Bounce(void *p)
{	
	time_t now; 
	now =time(NULL);
	ofstream out("E:/binding/" + int2string(now) + "-laser.txt");
	
	timebegin = GetCurrentStamp64();
	urg.set_sensor_time_stamp(0);
	urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);
	
	while( repeat != 'c' && repeat != 'q' )
	{
		vector<long> data;
		vector<unsigned short> intensity;
		//long time_stamp = 0;
		long time_stamp = time(NULL);
		//long time_stamp = static_cast<long>(XsTime::timeStampNow());
	

		if (!urg.get_distance_intensity(data, intensity, &time_stamp)) 
		{
			cout << "Urg_driver::get_distance(): " << urg.what() << endl;
			::_endthread();
			exit(1);
		}
		//	::Sleep(100L);可以利用来延迟操作

		print_data(urg, data, intensity, time_stamp , out, timebegin);
	}
	out.close();
	cout << "Laser-Completed One Time!" << endl;
	/*
#if defined(URG_MSC)
	getchar();
#endif
	*/
	::_endthread();
	
}

void CheckKey( void *dummy )
{
	SetThreadAffinityMask(::GetCurrentThread(), 0x00000004);
	while (1)
	{
		::Sleep(1);
		repeat = _getch();
		if ( repeat == 'q' )
			break;
	}
	::_endthread();
}

int main(int argc,char *argv[])
{
	time_t lt; 
	lt =time(NULL); 
	DWORD exitCode1 = 0;  
	///////////////////////
	DWORD exitCode2 = 0;
	////////////////////////////////////////////////
	repeat = 'c'; 
		///////////////////////////////////////
	::_beginthread( &CheckKey, 0, NULL );
	HANDLE p = NULL; //Laser-Thread
	HANDLE x = NULL; //Xsens-Thread
//--------------------------------------------------------------------------------------------------------//
	Connection_information information(argc,argv);

	if (!urg.open(information.device_or_ip_name(),
		information.baudrate_or_port_number(),
		information.connection_type())) {
			cout << "Urg_driver::open(): "
				<< information.device_or_ip_name() << ": " << urg.what() << endl;
			exit(0);
	}
#if 1
	urg.set_scanning_parameter(urg.deg2step(-Angel), urg.deg2step(+Angel), 0);
#endif
   /*	
	timebegin = GetCurrentStamp64();
	urg.set_sensor_time_stamp(0);
	urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);
	*/
//----------xSense----------------------------------------------------------------------------------------------//
	if (!setSerialKey())
	{
		std::cout << "Invalid serial key." << std::endl;
		std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();
		return 1;
	}

	// Create XsControl object
	std::cout << "Creating XsControl object..." << std::endl;
	XsControl* control = XsControl::construct();
	assert(control != 0);


	// Scan for connected devices
	std::cout << "Scanning for devices..." << std::endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi / MTx / MTmk4 device
	XsPortInfoArray::const_iterator mtPort = portInfoArray.begin();
	while (mtPort != portInfoArray.end() && !mtPort->deviceId().isMt9c() && !mtPort->deviceId().isMtMk4()) {++mtPort;}
	if (mtPort == portInfoArray.end())
	{
		throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
	}
	std::cout << "Found a device with id: " << mtPort->deviceId().toString().toStdString() << " @ port: " << mtPort->portName().toStdString() << ", baudrate: " << mtPort->baudrate() << std::endl;

	// Open the port with the detected device
	std::cout << "Opening port..." << std::endl;
	if (!control->openPort(mtPort->portName().toStdString(), mtPort->baudrate()))
	{
		throw std::runtime_error("Could not open port. Aborting.");
	}

	// Get the device object
	XsDevice* device = control->device(mtPort->deviceId());
	assert(device != 0);

	// Print information about detected MTi / MTx / MTmk4 device
	std::cout << "Device: " << device->productCode().toStdString() << " opened." << std::endl;

	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device in configuration mode
	std::cout << "Putting device into configuration mode..." << std::endl;
	if (!device->gotoConfig()) // Put the device into configuration mode before configuring the device
	{
		throw std::runtime_error("Could not put device into configuration mode. Aborting.");
	}

	// Configure the device. Note the differences between MTix and MTmk4
	std::cout << "Configuring the device..." << std::endl;
	if (device->deviceId().isMt9c())
	{
		XsOutputMode outputMode = XOM_Orientation; // output orientation data
		//XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion
		XsOutputSettings outputSettings = XOS_CalibratedMode_AccMagOnly;
		XsDeviceMode deviceMode(100); // make a device mode with update rate: 100 Hz
		deviceMode.setModeFlag(outputMode);
		deviceMode.setSettingsFlag(outputSettings);
		std::cout << "This is Mt9c-Xsens sensors" << endl;
		// set the device configuration
		if (!device->setDeviceMode(deviceMode))
		{
			throw std::runtime_error("Could not configure MTmki device. Aborting.");
		}
	}
	else if (device->deviceId().isMtMk4())
	{
		XsOutputConfiguration quat(XDI_Quaternion, 0);
		XsOutputConfiguration pose(XDI_GpsGroup, 0);
		XsOutputConfiguration vec(XDI_VelocityXYZ,0);

		XsOutputConfigurationArray configArray;

		configArray.push_back(quat);
		configArray.push_back(pose);
		configArray.push_back(vec);

		if (!device->setOutputConfiguration(configArray))
		{

			throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
		}
	}
	else
	{
		throw std::runtime_error("Unknown device while configuring. Aborting.");
	}

	// Put the device in measurement mode
	std::cout << "Putting device into measurement mode..." << std::endl;
	if (!device->gotoMeasurement())
	{
		throw std::runtime_error("Could not put device into measurement mode. Aborting.");
	}

	std::cout << "\nMain loop (press any key to quit)" << std::endl;
	std::cout << std::string(80, '-') << std::endl;
//--------------------------------------------------------------------------------------------------------//
	SYSTEM_INFO SystemInfo;
	GetSystemInfo(&SystemInfo);

	std::cout << SystemInfo.dwNumberOfProcessors << endl;
	/*
	timebegin = GetCurrentStamp64();
	urg.set_sensor_time_stamp(0);
	urg.start_measurement(Urg_driver::Distance_intensity, Urg_driver::Infinity_times, 0);
	*/
	while (1)
	{
		if (repeat == 'c')
		{
			::Sleep(1000L);

			std::cout << (lt=time(0)) <<"::"<< p << " : Waiting! " << GetExitCodeThread(p,&exitCode1) << std::endl;
			std::cout << (lt=time(0)) <<"::"<< x << " : Waiting! " << GetExitCodeThread(x,&exitCode1) << std::endl;
			if (GetExitCodeThread(p,&exitCode1) == 0 && GetExitCodeThread(x,&exitCode2) == 0) 
			{  
				p = (HANDLE)::_beginthread( &Bounce, 0, NULL );
				x = (HANDLE)::_beginthread( &Function, 0 ,&callback);
				SetThreadAffinityMask(p, 0x00000002);
				SetThreadAffinityMask(x, 0x00000003);
			}

			::SuspendThread(x);
			::SuspendThread(p);
			
			std::cout << (lt=time(0)) <<" Laser-Thread::"<< p << " : Waiting! " << GetExitCodeThread(p,&exitCode1) << std::endl;
			std::cout << (lt=time(0)) <<" Xsens-Thread::"<< x << " : Waiting! " << GetExitCodeThread(x,&exitCode1) << std::endl;
			std::cout << std::endl;

			continue;
		}
		if ( repeat == 'q')
		{
			std::cout << "Reach The Finished  Line" << std::endl;
			break;
		}
		else
		{
			::ResumeThread(p);
			::ResumeThread(x);
		}
	}

	std::cout << "Closing port..." << std::endl;
	control->closePort(mtPort->portName().toStdString());

	// Free XsControl object
	std::cout << "Freeing XsControl object..." << std::endl;
	control->destruct();

	std::cout << "Successful exit." << std::endl;

	return 0;
}
