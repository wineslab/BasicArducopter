###################################################################################################
# Copyright © Abhimanyu Venkatraman Sheshashayee <abhi.vs@outlook.com>. All rights reserved.      #
# Please refer to the LICENCE file in the project root for information on use and distribution.   #
# Unauthorised use of this file is strictly prohibited.                                           #
# Unauthorised distribution of this file, via any medium, is strictly prohibited.                 #
###################################################################################################

### This module is dependent on the 'BasicArducopter' project by John Buczek.

### Imports ---------
from BasicArducopter.BasicArdu.BasicArdu import BasicArdu, Frames

### Main Function ----------
def main():
	
	drone = BasicArdu( connection_string = '/dev/ttyACM0' )
	drone.handle_landing()

### Program entry point ----------
if __name__ == '__main__':
	main()