////////////////////////////////////////////////////////////////////////////////////////////
// \file 	RoutingInstructionList.h
//
// \author 	Anthony Baker
// \date 	2018/07/30
//
// \brief 	Interface for additional routing instructions 
//
// \note 	The interface for routing instructions
////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Interface/Serializer.h"

namespace Boeing
{
	namespace Router
	{
		class RoutingInstructionList : public interface::Serializer
		{
		public:
			// ================================================================================
			// \fn:       Boeing::Router::RoutingInstruction
			// \brief:    Default Constructor
			//
			// @return    
			// ================================================================================
			RoutingInstructionList();
      
            virtual void JsonSerialize(Json::Value & jsonNode) const override;
            virtual void JsonDeserialize(const Json::Value & jsonNode) override;
		
		};
	}
}