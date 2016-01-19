# The QSR Probabilistic Representation library

This library provides functionalities to create probablistic models of QSR state chains, e.g. produced by the qsr_lib. This can for example be used for classification and sampling of new state chains. Currently only HMMs are supported.

## Usage

The recommended language is python 2.7 using ROS Indigo. Eventhough the library uses ROS only for communication, the provided infrastructure currently relies on ROS services to be used. Python is recommended due to the fact that the provided ros_client is implemented in python and using the services directly might be a bit confusing at first since they rely on json parsable strings.

For a python usage exmple, please see the example_ros_client.py in scripts. Try `rosrun qsr_prob_rep example_hmm_client.py -h` or `rosrun qsr_prob_rep example_pf_client.py -h` for usage information depending on if you want to create a Hidden Markov Model or a Particle Filter.

### Currently implemented functionality

**HMM**

* _create_: This takes the desired qsr_type and a json parsable list of lists of QSR state chains and returns the xml representation of the trained HMM as a string. This function is easiest to use when reading the state chains from files as it's done in the example client. The resulting xml can either be written to disk, kept in memory, or stored in a datacentre. The xml string is used in all other functionalities to load the HMM.
* _sample_: Given a HMM as an XML string, the desired number and length of samples, and the qsr the HMM models, this function produces sample state chains and returns them as numpy arrays.
* _loglikelihood_: Given an HMM and a (list of) state chain(s), this function calculates the accumulative loglikelihood for the state chain(s) to be produced by the given HMM. Might produce `-inf` if production is impossible.

**Particle Filter**

* _create_: This takes a model consisting or several transition probability and observation probability matrices in a dictionary and a state look up table to create a particle filter. Create an instance of the `PfRepRequestCreate` class filling all the necessary information. The required model can be built with a helper class `qsrrep_pf.pf_model.PfModel` that includes tests for the model sanity. Have a look at the example client on how to use this. The look up table has to have as many states as the matrices have rows and coloumns, and should be a simple list or numpy array of the states that will be observed. The index of the state in the look up table has to correspond to the index for this state in the given matrices. Have a look at the `qsrrep_utils.qtc_model_generation` class for inspiration on how to create one. This returns a uuid identifying your particle filter.
 * The necessary files can be create directly from the HMM by exporting the emission matrix as the observation probability matrix and the transitions as the transition probability matrix. This however, requires that you have enough training data to learn sensible emissians and transitions at the same time. If not, create your own observation probability matrix like it is done in `qsrrep_utils.qtc_model_generation`. The `create_pf_models.py` script can help to generate the files from the HMM.
* _predict_: Takes the uuid of an existing particle filter and the number of sample generations to predict and returns the most likely next states and model including their probabilities.
* _update_: Runs the baysian update for the particle filter identified by uuid id using an observation provided.
* _list_filters_: Lists all currently active filters.
* _remove_: Removes the particle filter with the given uuid from memory.

**General usage advice**

The ros_client for python hides a lot of the complexity from the user. Just create an instance of the correct request class and call `ROSClient().call_service` and the right service will be called for you. All the json parsing happens in the background and you don't have to worry about this. Have a look at the example clients to see how to do this.

### Currently implemented QSRs

* _QTCB_: The basic variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]
* _QTCC_: The double-cross variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]
* _QTCBC_: The mixture representation of the basic and double-cross variant of the Qtalitative Trajectory Calculus. For more information on the implementation see [1]
* _RCC3_: A very simple rcc3 representation using uniform transition and emission matrices.

## For Developers

**The following is currently hevily outdated and needs updating!!!**

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
