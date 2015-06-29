# The QSR HMM representation library

This library provides functionalities to create HMM models of QSR state chains, e.g. produced by the qsr_lib. This can for example be used for classification and sampling of new state chains.

## Usage

The recommended language is python 2.7 using ROS Indigo. Eventhough the library uses ROS only for communication, the provided infrastructure currently relies on ROS services to be used. Python is recommended due to the fact that the provided ros_client is implemented in python and using the services directly might be a bit confusing at first since they rely on json parsable strings.

For a python usage exmple, please see the example_ros_client.py in scripts. Try `rosrun qsr_hmm_rep example_ros_client.py -h` for usage information.

### Currently implemented functionality

* _Create_: This takes the desired qsr_type and a json parsable list of lists of QSR state chains and returns the xml representation of the trained HMM as a string. This function is easiest to use when reading the state chains from files as it's done in the example client. The resulting xml can either be written to disk, kept , or stored in a datacentre. The xml string is used in all other functionalities to load the HMM.
* _Sample_: Given a HMM as an XML string, the desired number and length of samples, and the qsr the HMM models, this function produces samples state chains and returns them as numpy arrays.
* _Loglikelihood_: Given an HMM and (a list of) state chain(s), this function calculates the accumulative loglikelihood for the state chain(s) to be produced by the given HMM.

### Currently implemented QSRs

* _QTCB_: The basic variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]
* _QTCC_: The double-cross variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]
* _QTCBC_: The mixture representation of the basic and double-cross variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]

## For Developers

### Adding a new QSR

TODO: Implement arg_distance as an easy example

### Adding a new functionality

* Add you new functionality to `hmmrep_hmms.hmm_abstractclass.py` following the examples of `_create`, `_sample`, and `_log_likelihood`.
  * Add a getter for your new functionality in `hmmrep_hmms.hmm_abstractclass.py` following the examples of `get_hmm`, `get_samples`, and `get_log_likelihood`.
* Create a request class following the naming scheme: `HMMRepRequestFunctionality` in `hmmrep_lib.hmmrep_io.py` where 'Functionality' is replaced by your new functionality name.
  * Inherit from `HMMRepRequestAbstractclass`
  * Define the `_const_function_pointer` to use your function in `hmmrep_lib.py`.
    * Make th pointer look like the one in `HMMRepRequestAbstractclass` and replace `my_function` with the function name in `hmmrep_lib.py` (implemented later on)
    * Override `__init__` definig a custom function header and adding the variables to the variable `self.kwargs`, following the example of the other classes in this file.
* Create a response class in `hmmrep_lib.hmmrep_io.py` following the naming scheme 'HMMReqResponseFunctionality' where 'Funtionality' should be the same as for the request class.
  * Override the 'get' function to make sure it returns a string (str or json dump)
* Add your new functionality to 'available_services' in the bottom of `hmmrep_lib.hmmrep_io.py`.
    * The string key will be used to create the service name
    * The value should be a list where the first entry is your request class and the second the response class.
* Add a new function in `hmmrep_lib.hmmrep_lib.py` that calls your getter function from `hmmrep_hmms.hmm_abstractclass.py` and returns your response from `hmmrep_lib.hmmrep_io.py`
* The ros server will automatically create a service for your new functionality and the ros_client will now how to deal with it given you request class is used as an input for it.
* Add thee new functionality to the `example_ros_client.py` using proper argument parsing like done for the other functions.




[1] Dondrup, C.; Bellotto, N.; Hanheide, M.; Eder, K.; Leonards, U. A Computational Model of Human-Robot Spatial Interactions Based on a Qualitative Trajectory Calculus. In: Robotics 2015, 4, 63-102.
