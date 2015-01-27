^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qsr_lib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.0.2 (2015-01-27)
------------------

0.0.1 (2015-01-27)
------------------
* Adding proper install targets and adjusting package.xml
* Update README.md
* Added .md to readme to make it markdown
* Fixing spelling mistake and adding QTC description to README
* Removing some prints and explicitly setting quantisation_factor to 0 if None in the make function.
* Adding custom test to see if x and y are defined.
* Adding an optional argument to omit collapsing qtc states.
  Adding ability to read a csv file with xy coordinates for the qtc representation
* Adding collapse functionality to collapse similar adjacent states
* nan value for empty fields in world_trace/Object_State
* Added the option of reading the incoming object data from a file using the example ros client to make it easier to test QSRs
  Does not change the the default behaviour.
  Also added a few optional arguments just for qtc.
* Added correct validation function
* Added:
  * validation argument
  * quantisation_factor argument
  * custom help
  * custom check
  Bugs: Validation is not working correctly yet. Only accounts fro transitions from -1 to 1 and vice-versa. Many more constraints to be added.
* Added quantisation factor
* Added new line at end of file
* * Adding QTCC
  * the abstract class now contains all the relevant code. The implementing classes only select the part of the QTCC tuple they want to return and implement the custom_* functions
* Added parent class to compute qtc states from incoming position arrays.
  Computes qtcc, for qtcb the last two values can simply be omitted.
  Issues:
  * Has only been tested for the distance constraint so far
  * Needs validation of state chains, this includes the insertion of virtual timestamps
  * So far there is no need for it to be abstract, might be changed in future
  * Quantisation factor has to be given together with the data
* QSRlib, uses new IO, major change
* removed timestamps list
* removed legacy files
* check in
* python new qsrlib ready
* check in
* legacy tidy up
* check in
* check in
* breaking
* moving
* check in
* qtc_b in progress -bugged euclidean
* added support for reusing previous passed input_data (if wanted)
* Various and very importants (see comments below for details)
  1) Renamed class QSR_Lib to QSRlib
  2) Renamed request methods in QSRlib and in the QSRlib_ROS_Server
  to have the same name "request_qsrs"
  3) Renamed qsr_lib_ros_* to qsrlib_ros_*
  4) Provided an example of using the QSRlib_ROS_Client for ease of
  read
  5) README updated to reflect the changes
* fixed a bug on example.py
* qsrlib instracture + example RCC3 2D rectangles for devs
* Contributors: Christian Dondrup, Yiannis Gatsoulis
