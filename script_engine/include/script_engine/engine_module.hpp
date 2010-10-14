#ifndef ENGINE_MODULE_HPP
#define ENGINE_MODULE_HPP

#include "engine_v8.hpp"
#include <v8.h>

namespace script_engine
{
	/// \brief A plugin module for the V8 execution engine.
	class engine_module
	{
	public:
		engine_module() {};
		virtual ~engine_module() {};

		/// \brief Called by the script engine once ready.
		///
		/// In this method, you should register functions and objects in the
		/// global object. 
		virtual void init(v8::Handle<v8::ObjectTemplate>& global) = 0;

	};

}

#endif

