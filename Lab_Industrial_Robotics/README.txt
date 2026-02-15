The files in this folder are all the necessary ones to run the code. 
The code to be run to start the UI and perform the movements is the one named "main.py". Such file exploits variables, classes and function defined in other scripts. 
The auxiliary files are divided as follows:

- libraries: 	

	- "onrobot_tm.py"
	- "rich_logging_format.py"
	- "techman.py"
	- "tm_motion_functions_V1.py"
	- "tm_packet"

- constants needed throughout the whole execution:

	- "constants.py"

- computation of motion parameters, steps of the movement and movement sequences to perform:

	- "motion_parameters.py"
	- "motion_sequences"
	- "motion_steps.py"

- creation of the user interface and application used to choose the helmet and choose and launch each movement:

	- "user_interface.py"


The code has been divided in multiple files to enhance its readability and understandability.

However, it is still present a file containing everything in a single script, which is named "complete_motion_UI.py"