// Copyright (C) 2020  Max Coutte, Gabriel Combe
// Copyright (C) 2020  Relativty.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "User32.lib")
#pragma comment (lib, "Setupapi.lib")
#include <C:\Program Files\Python310\include\Python.h>

#include <iostream>
#include <filesystem>
#include <string>
#include "Relativty_ServerDriver.hpp"

namespace fs = std::filesystem;

void startPythonTrackingClient_threaded(std::string PyPath) {
	PyObject* module, * dict, * python_class, * sample_object;
	_putenv_s("PYTHONPATH", "D:/CODE/PYTHONPATH/");
	Py_Initialize();

	module = PyImport_ImportModule("foobar");
	if (module == nullptr)
	{
		std::cout << "Failed to import module.";
		//return 1;
	}

	dict = PyModule_GetDict(module);
	if (dict == nullptr)
	{
		std::cout << "Failed to get module dict.";
		//return 1;
	}
	Py_DECREF(module);

	python_class = PyDict_GetItemString(dict, "foobar");
	if (python_class == nullptr)
	{
		std::cout << "Failed to get class.";
		//return 1;
	}
	Py_DECREF(dict);

	sample_object = PyObject_CallObject(python_class, nullptr);
	if (sample_object == nullptr)
	{
		std::cout << "Failed to instantiate object.";
		//return 1;
	}
	Py_DECREF(python_class);

}