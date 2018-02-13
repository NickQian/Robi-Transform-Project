/*
  Interface between ap.py and C driver of the alsa mic/Headphone on pi2/3
  ----
  Licensed under BSD license
  0.1: 2016.10.21. Init version by Nick Qian. Extract from old "robiClib.cpp"
  ----
  Input: Python call
  Output: To stt and tts module 
*/

#include <iostream>
#include <bitset>
#include <stdlib.h>
#include <stdint.h>           // C99 int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t;

#include "audioInIF.h"
//#include "peri.h"
//#include "futabaServo.h"
#include "tts.h"
#include "stt.h"
#include "mic.h"

using namespace std;


/*---------------------------Python Interface--------------------------------*/
//b (int)-> [unsigned char]  -refer: doc->Exgtending and Embedding the Python I nterpreter
//B (int)-> [unsigned char] -without overflow checking
//c (string) -> [char]
//h (int)-> [short int]
//H (int)  unsigned short int]
//i (int)->  int]
//I (int)-> unsigned int]
//s(string) to [char *] Python string to a C pointer to a character string 


static PyObject* _tts(PyObject *self, PyObject *args){
  int result;
  const char *text;
  const char *tts_args;
  PyObject *retval;

  cout << "---Py calling tts ---" << endl;

  if ( !PyArg_ParseTuple(args, "s", &text) ){   // convert Python -> C
   cout << "Error during PyArg_ParseTuple" << endl; 
          return NULL;
  }
  result = tts(text);      //, tts_args);

  retval = (PyObject *)Py_BuildValue("i", result);      // convert C -> Python 

  return retval;
}

static PyObject* _stt(PyObject *self, PyObject *args){
  const char * text_result;
  string str_stt_result;

  PyObject *retval;

  cout << "---Py Calling stt---" << endl;
 // if (!PyArg_ParseTuple(args, "s", &textRtn) ){    // Python -> C
 //    return NULL;
 // }

  text_result = stt();

  retval =  (PyObject *)Py_BuildValue("s", text_result);   // C-> Python

  str_stt_result = text_result;
  cout << "@@STRING stt_result is:" << str_stt_result << endl;

  return retval;
}

static PyObject* _micStartListen(PyObject *self, PyObject *args){
  int result;
  PyObject *retval;

  cout << "====Py Calling mic listen=====" << endl;
 // if (!PyArg_ParseTuple(args, "s", &textRtn) ){    // Python -> C
 //    return NULL;
 // }

  result = micStartListen();
  retval =  (PyObject *)Py_BuildValue("i", result);   // C-> Python

  return retval;
}


static PyObject* _initAlsaMicrophone(PyObject *self, PyObject *args){
  int result;
  PyObject *retval;

  cout << "======Py Calling initAlsaMicrophone======" << endl;
 // if (!PyArg_ParseTuple(args, "s", &textRtn) ){    // Python -> C
 //    return NULL;
 // }

  result = initAlsaMicrophone();
  retval =  (PyObject *)Py_BuildValue("i", result);   // C-> Python

  return retval;
}

static PyObject* _mop_Up_Mic(PyObject *self, PyObject *args){
  int result;
  PyObject *retval;

  cout << "======Py Calling mop-up_Mic =======" << endl;
 // if (!PyArg_ParseTuple(args, "s", &textRtn) ){    // Python -> C
 //    return NULL;
 // }

  result = mop_Up_Mic();
  retval =  (PyObject *)Py_BuildValue("i", result);   // C-> Python

  return retval;
}

///////////////refer: doc->Extending and Embedding the Python Interpreter//////////////
///////PyArg_ParseTuple////////
//s (string)  -> [const char *] Python string to a C pointer to a character string 
//s# (string) -> [cosnt char *, int]
//b (int)     -> [unsigned char]
//B (int)     -> [unsigned char] -without overflow checking
//c (string)  -> [char]
//h (int)     -> [short int]
//H (int)     -> [unsigned short int]
//i (int)     -> [int]
//I (int)     -> [unsigned int]
/////////PyBuildValue//////////
//s (string) -> [char *]  convert C string to Python object
//s# (string) ->[char *, int] convert C string and it's len to Python object


//////////////// registration table struct//////////////////
static struct PyMethodDef audioInIFMethods[]={
  {"tts",                _tts,                METH_VARARGS, "calling tts"                           },
  {"stt",                _stt,                METH_VARARGS, "call stt"                              },
  {"micStartListen",     _micStartListen,     METH_VARARGS, "call mic start listen"                 },
  {"initAlsaMicrophone", _initAlsaMicrophone, METH_VARARGS, "do init microphone task"               },
  {"mop_Up_Mic",         _mop_Up_Mic,         METH_VARARGS, "do mop-up microphone work"             },

  {NULL, NULL, 0, NULL}
};

/////////////////// module initializer ///////////////////
#ifdef Python3
static struct PyModuleDef Periperalmodule ={
	PyModuleDef_HEAD_INIT,
	"Periperalmodule",
	NULL,
	-1,
	PeriperalMethods
};


PyMODINIT_FUNC PyInit_robiClib(void) {
    //(void)Py_InitModule("Periperal", PeriperalMethods); 
	return PyModule_Create(&Periperalmodule);
}

#else

PyMODINIT_FUNC initlibaudioInIFlib(void){
   (void) Py_InitModule("libaudioInIFlib", audioInIFMethods);
}

#endif  


