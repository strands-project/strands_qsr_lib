^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qsr_prob_rep
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2019-01-25)
------------------
* Typo in boolean (`#244 <https://github.com/strands-project/strands_qsr_lib/issues/244>`_)
* Solved casting issue. Testing on kinectic (`#243 <https://github.com/strands-project/strands_qsr_lib/issues/243>`_)
* Updated email to new Oxford address
* Contributors: Manuel Fernandez-Carmona, Nick Hawes

0.4.0 (2017-09-01)
------------------
* changed maintainer to marc
* A little bit of error handling for incorrect model and state numbers. (`#241 <https://github.com/strands-project/strands_qsr_lib/issues/241>`_)
* Merge pull request `#236 <https://github.com/strands-project/strands_qsr_lib/issues/236>`_ from cdondrup/noxml
  Removing the xml overhead from the HMM representations
* Merge pull request `#3 <https://github.com/strands-project/strands_qsr_lib/issues/3>`_ from pet1330/noxml
  member variable init when constructed
* member variable init when constructed
* Transforming test HMMs into new format.
  Adapted test script to work with new interface.
* Removing the hmm types active
* Adding script to transform old format HMMs into new format HMMs
* Merge pull request `#2 <https://github.com/strands-project/strands_qsr_lib/issues/2>`_ from pet1330/noxml
  Making the hmm lib thread safe. Especially for the generic HMM.
* Making the hmm lib thread safe. Especially for the generic HMM.
* Adapting example client to now hmm format.
* Removing xml overhead form hmm library. This was horrid to begin with.
  Using a dictionary of lists now.
* Contributors: Christian Dondrup, Marc Hanheide, Peter Lightbody

0.3.0 (2016-01-20)
------------------
* Fixing normalisation of matrix. It only worked because they are square and all entries are 1.
* Printing debug information on creation of the pf for all models not only for the first two. Might have also killed the server if only one model.
* Adding all the new arguments of the hmm create message to the example hmm client. Use ... create -h for help.
* The assumption that every hmm has a start state 0 is not valid. now there is a flag `start_at_zero` when you want to enforce a start state at 0.
* generic hmm emi and trans have to numpy arrays to be consistent.
* Fixing available hmms for help message
* Adding simple tests for the particle filter
* Fixing install targets.
* Adding pybayes as a dependency
* Adding optional pseudo transitions to hmm.
* Adding a generic hmm that takes a lookup table, and optionally a transition and emission matrix. If the latter two are not provided, uniform distribution is assumed.
* Updating readme slightly
* Adding example files for particle filter creation.
* Adding helper functions and scripts to create all the files necessary for the particle filter from the HMM or for QTC
* Adding a particle filter for QRSs
* * Decoupling HMMs from the main library functionality to allow for other representations to be added more easily.
  * The Services are now create by decorating functions using the ServiceManager class.
  * Simplifies the service generation by not needing to maintain a dictionary with all the services.
  * Simplifies the class structure by not calling weird lambda functions anymore.
  * removing `qsr_type` from the service request and response.
  * Now part of the general kwargs for hmm
  * Particle filter will not use this
  * Allows to have a generic hmm that takes trans and emi matrix plus a lookup instead of a specific implementation.
* Renamed example ros client to example hmm client as every representation will have their own client.
* Introducing namespaces for services to distinguish between hmm and other representations.
* Contributors: Christian Dondrup

0.2.1 (2015-12-08)
------------------
* Waiting for services to become available
* Contributors: Christian Dondrup

0.2.0 (2015-08-27)
------------------

0.1.3 (2015-08-13)
------------------

0.1.2 (2015-08-11)
------------------
* Adding RCC3 as an easy example on how to add new QSRs to the prob rep lib.
  Also, updating README.
* Removing rubbish from test file.
* Updated documentation to reflect name changes.
* Cleaning up cmake and package file, adding qsr_prob_rep to meta package
* Adding tests for creation, sampling and loglikelihood of all three qtc types.
* Finally enabling qtcbc and minor changes
  * qtcbc now works using code from qtcb and qtcc and some custom functions overwriting the ones in qtc_hmm_abstractclass
  * Removed some prints and fixed comments.
* Fixing a bug where the final state was always 82... d'oh
* Renaming package
* Contributors: Christian Dondrup

0.1.1 (2015-06-19)
------------------

0.1.0 (2015-06-16)
------------------

0.0.8 (2015-05-17)
------------------

0.0.7 (2015-04-22)
------------------

0.0.6 (2015-03-04)
------------------

0.0.5 (2015-02-27)
------------------

0.0.4 (2015-02-26)
------------------

0.0.3 (2015-01-27 20:25)
------------------------

0.0.2 (2015-01-27 16:55)
------------------------

0.0.1 (2015-01-27 14:04)
------------------------
