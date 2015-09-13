#include "stdafx.h"
#include "CppUnitTest.h"

#include "garmin_wrapper.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTestGarmin
{		
	TEST_CLASS(Basics)
	{
	public:
		
		TEST_METHOD(TestOpen)
		{
            Garmin_GPS_Open("gps");
		}

	};
}