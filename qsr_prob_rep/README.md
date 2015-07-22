# The QSR Probabilistic Representation library

This library provides functionalities to create probablistic models of QSR state chains, e.g. produced by the qsr_lib. This can for example be used for classification and sampling of new state chains. Currently only HMMs are supported.

## Usage

The recommended language is python 2.7 using ROS Indigo. Eventhough the library uses ROS only for communication, the provided infrastructure currently relies on ROS services to be used. Python is recommended due to the fact that the provided ros_client is implemented in python and using the services directly might be a bit confusing at first since they rely on json parsable strings.

For a python usage exmple, please see the example_ros_client.py in scripts. Try `rosrun qsr_prob_rep example_ros_client.py -h` for usage information.

### Currently implemented functionality

* _create_: This takes the desired qsr_type and a json parsable list of lists of QSR state chains and returns the xml representation of the trained HMM as a string. This function is easiest to use when reading the state chains from files as it's done in the example client. The resulting xml can either be written to disk, kept in memory, or stored in a datacentre. The xml string is used in all other functionalities to load the HMM.
* _sample_: Given a HMM as an XML string, the desired number and length of samples, and the qsr the HMM models, this function produces sample state chains and returns them as numpy arrays.
* _loglikelihood_: Given an HMM and a (list of) state chain(s), this function calculates the accumulative loglikelihood for the state chain(s) to be produced by the given HMM. Might produce `-inf` if production is impossible.

### Currently implemented QSRs

* _QTCB_: The basic variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]
* _QTCC_: The double-cross variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]
* _QTCBC_: The mixture representation of the basic and double-cross variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]
* _RCC3_: A very simple rcc3 representation using uniform transition and emission matrices.

## For Developers

### Adding a new QSR

Adding a new QSR based on the simple RCC3 example:

* Create a new file in `src/qsrrep_hmms` or copy the `rcc3_hmm.py`
* Import: `from qsrrep_hmms.hmm_abstractclass import HMMAbstractclass`
* Create a new class that inherits from `HMMAbstractclass`
* In the `__init__(self)` function:
 * Call the super calss constructor: `super(self.__class__, self).__init__()`
 * Set `self.num_possible_states` to the number of possible states your QSR has
* Overwrite the two functions `_qsr_to_symbol` and `_symbol_to_qsr`. See rcc3 example for inspiration.
* Add your new QSR to `src/qsrrep_lib/rep_lib.py`:
 * Import your new file from `qsrrep_hmms`
 * Add your new QSR to the dictionary of available QSRs `hmm_types_available` following the examples given
* Enjoy your new QSR HMM!

The example client is agnostic of the QSR implemented, a simple restart of the eserver and allows to use your QSR with the example client immediately.

### Adding a more specialised QSR

Sometimes a uniformly distributed transition and emission matrix (as they are used in above example by default) is just not precise enough like in the case of QTC. Please have a look at `src/qsrrep_hmms/qtc_hmm_abstractclass.py` on how to deal with that.

Basic instructions:

* Follow above instructions to create a new QSR
* Overwrite the `_create_transition_matrix` and `_create_emission_matrix`

### Adding a new functionality

* Add your new functionality to `qsrrep_hmms.hmm_abstractclass.py` following the examples of `_create`, `_sample`, and `_log_likelihood`.
  * Add a getter for your new functionality in `qsrrep_hmms.hmm_abstractclass.py` following the examples of `get_hmm`, `get_samples`, and `get_log_likelihood`.
* Create a request class following the naming scheme: `HMMRepRequestFunctionality` in `qsrrep_lib.rep_io.py` where `Functionality` is replaced by your new functionality name.
  * Inherit from `HMMRepRequestAbstractclass`
  * Define the `_const_function_pointer` to use your function in `rep_lib.py`.
    * Make the pointer look like the one in `HMMRepRequestAbstractclass` and replace `my_function` with the function name in `rep_lib.py` (implemented later on)
    * Override `__init__` definig a custom function header and adding the variables to the variable `self.kwargs`, following the example of the other classes in this file.
* Create a response class in `qsrrep_lib.rep_io.py` following the naming scheme `HMMReqResponseFunctionality` where `Funtionality` should be the same as for the request class.
  * Override the `get` function to make sure it returns a string (str or json dump)
* Add your new functionality to `available_services` in the bottom of `qsrrep_lib.rep_io.py`.
    * The string key will be used to create the service name
    * The value should be a list where the first entry is your request class and the second the response class.
* Add a new function in `qsrrep_lib.rep_lib.py` that calls your getter function from `qsrrep_hmms.hmm_abstractclass.py` and returns your response from `qsrrep_lib.rep_io.py`
* The ros server will automatically create a service for your new functionality and the ros_client will know how to deal with it given your request class is used as an input for it.
* Add thee new functionality to the `example_ros_client.py` using proper argument parsing like done for the other functions.




[1] Dondrup, C.; Bellotto, N.; Hanheide, M.; Eder, K.; Leonards, U. A Computational Model of Human-Robot Spatial Interactions Based on a Qualitative Trajectory Calculus. In: Robotics 2015, 4, 63-102.
