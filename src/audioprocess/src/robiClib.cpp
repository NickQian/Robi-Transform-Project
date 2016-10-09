//
// robiClib.cpp : Defines the exported functions for the DLL application.
//

#include <iostream>
#include <bitset>
#include <stdlib.h>
#include <stdint.h>           // C99 int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t;

#include "robiClib.h"
#include "peri.h"
#include "futabaServo.h"
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

static PyObject * _writeAddr32(PyObject *self, PyObject *args){
  int result;
  PyObject *retval;    // for Func return
  volatile uint32_t *addr;
  int32_t wr_data;

  if(!PyArg_ParseTuple(args,"si", &addr, &wr_data) ){   // convert Python -> C 
	  return NULL;
  }

  result = writeAddr32(addr, wr_data);                  // Func
  retval = (PyObject *)Py_BuildValue("i", result);      // convert C -> Python

  //return PyLong_FromLong(result);
  return retval;
}

static PyObject * _readAddr32(PyObject *self, PyObject *args){
  int result;
  volatile uint32_t *addr;
  PyObject * retval;                 // for return

  if(!PyArg_ParseTuple(args, "s", &addr) ){            // convert Python -> C 
     return NULL;
  }

  result = readAddr32(addr);
  retval = (PyObject *) Py_BuildValue("i", result);     // convert C -> Python

  return retval;
  //return PyLong_FromLong(result);
}


static PyObject* _servoSet(PyObject *self, PyObject *args){
	uint8_t result;
	uint8_t servoID=1;
	int16_t goalPostion;
	uint16_t goalTime;
	PyObject *retval;

        cout << "=========PyObject calling servoSet=========" << endl;

	if(!PyArg_ParseTuple(args, "bhH", &servoID, &goalPostion, &goalTime) ){ // convert Python -> C 
		return NULL;
	}
	result = servoSet(servoID, goalPostion, goalTime);
	retval = (PyObject *) Py_BuildValue("b", result);       // convert C -> Python 

	return retval;
}


static PyObject* _peri_Init(PyObject *self, PyObject *args){
  int result;
  uint32_t baudrate_spi;
  uint32_t baudrate_uart1;
  uint32_t baudrate_pwmser;
  PyObject *retval;

  cout << "==============PyObject calling peri_Init===================" << endl;

  if (!PyArg_ParseTuple(args, "III", &baudrate_spi, &baudrate_uart1, &baudrate_pwmser) ){   // convert Python -> C 
	  return NULL;
  }

  result = peri_Init(baudrate_spi, baudrate_uart1, baudrate_pwmser );
  retval = (PyObject *)Py_BuildValue("i", result);      // convert C -> Python 
  return retval;
}


static PyObject* _GPIO_PWM_Set(PyObject *self, PyObject *args){
  int result;
  int ch_num=1;
  int postion;
  PyObject *retval;

  if (!PyArg_ParseTuple(args, "i", &ch_num) ){          // convert Python -> C 
	  return NULL;
  }

  result = PWM_Set(ch_num, postion);
  retval = (PyObject *)Py_BuildValue("i", result);      // convert C -> Python 

  return retval;
}

static PyObject* _UART0_Init(PyObject *self, PyObject *args){	
  int result;
  int baudRate;
  PyObject *retval;

  if (!PyArg_ParseTuple(args, "i", &baudRate) ){   // convert Python -> C 
	  return NULL;
  }

  result = UART0_Init(baudRate);
  retval = (PyObject *)Py_BuildValue("i", result);      // convert C -> Python 

  return retval;
  }

static PyObject* _UART0_Send(PyObject *self, PyObject *args){
  int result;
  unsigned char uartTxData;
  PyObject *retval;

  if (!PyArg_ParseTuple(args, "b", &uartTxData) ){   // convert Python -> C 
	  return NULL;
  }

  result = UART0_Send(uartTxData);
  retval = (PyObject *)Py_BuildValue("i", result);      // convert C -> Python 

  return retval;
}

static PyObject* _servoSetID(PyObject *self, PyObject *args){
  int result;
  int16_t id;
  PyObject *retval;
cout << "======Py calling servoSetID =======" << endl;

  if (!PyArg_ParseTuple(args, "h", &id) ){   // convert Python -> C 
          return NULL;
  }

  result = servoSetID(id);
  retval = (PyObject *)Py_BuildValue("i", result);      // convert C -> Python 

  return retval;
}

static PyObject* _tts(PyObject *self, PyObject *args){
  int result;
  const char *text;
  const char *tts_args;
  PyObject *retval;

  cout << "=======Py calling tts =========" << endl;

  if ( !PyArg_ParseTuple(args, "s", &text) ){   // convert Python -> C
   cout << "Error during PyArg_ParseTuple" << endl; 
          return NULL;
  }
  result = tts(text);      //, tts_args);

  retval = (PyObject *)Py_BuildValue("i", result);      // convert C -> Python 

 // cout << "====End tts Interface===" << endl;
  
  return retval;
}

static PyObject* _stt(PyObject *self, PyObject *args){
  const char * text_result;
  string str_stt_result;

  PyObject *retval;

  cout << "=======Py Calling stt===========" << endl;
 // if (!PyArg_ParseTuple(args, "s", &textRtn) ){    // Python -> C
 //    return NULL;
 // }

  text_result = stt();

  retval =  (PyObject *)Py_BuildValue("s", text_result);   // C-> Python

  str_stt_result = text_result;
  cout << "@@@@STRING str_stt_result is:" << str_stt_result << endl;

  //cout << "====End stt Interface======" << endl;

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

 // cout << "====End mic listen Interface======" << endl;

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
static struct PyMethodDef PeriperalMethods[]={
  {"writeAddr32",    _writeAddr32,   METH_VARARGS, "address based register/memory write"   }, //METH_NOARGS
  {"readAddr32",     _readAddr32,    METH_VARARGS, "address based register/memory write"   },
  {"servoSet",       _servoSet,      METH_VARARGS, "Set Servo with ID, postion and speed"  },  
  {"peri_Init",      _peri_Init,     METH_VARARGS, "initialize the peripherals"            },
  {"GPIO_PWM_Set",   _GPIO_PWM_Set,  METH_VARARGS, "Set the PWM registers to desired value"},
  {"UART0_Init",     _UART0_Init,    METH_VARARGS, "initialize the GPIO as UART0"          },
  {"UART0_Send",     _UART0_Send,    METH_VARARGS, "UART0 send data"                       },
  {"servoSetID",     _servoSetID,    METH_VARARGS, "Set Futaba servo ID"                   },
  {"tts",            _tts,           METH_VARARGS, "calling tts"                           },
  {"stt",            _stt,           METH_VARARGS, "call stt"                              },
  {"micStartListen", _micStartListen,METH_VARARGS, "call mic start listen"                 },

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

PyMODINIT_FUNC initrobiClib(void){
   (void) Py_InitModule("robiClib", PeriperalMethods);
}

#endif  


