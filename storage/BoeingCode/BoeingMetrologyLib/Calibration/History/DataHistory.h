#ifndef BOEINGMETROLOGYLIB_DATAHISTORY_H
#define BOEINGMETROLOGYLIB_DATAHISTORY_H

#include <string>
#include <cv.h>
#include "Common/Interface/Serializer.h"
#include "BoeingMetrologyLib_API.h"
#include "TypeDefs.h"
#include <map>
namespace BoeingMetrology
{
namespace Calibration
{
	class BOEINGMETROLOGYLIB_API DataHistory : public Boeing::Interface::Serializer
	{
	public:
		virtual void WriteToCSV(std::ostream& csvStream)
		{
			return;
		}
	};//class DataHistoryWrapper

}//namespace Calibration
}//namespace BoeingMetrology

#endif