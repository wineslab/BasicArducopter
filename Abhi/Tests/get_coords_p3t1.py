###################################################################################################
# Copyright © Abhimanyu Venkatraman Sheshashayee <abhi.vs@outlook.com>. All rights reserved.      #
# Please refer to the LICENCE file in the project root for information on use and distribution.   #
# Unauthorised use of this file is strictly prohibited.                                           #
# Unauthorised distribution of this file, via any medium, is strictly prohibited.                 #
###################################################################################################

### This module is dependent on the 'BasicArducopter' project by John Buczek.

### Imports ----------
import os
from BasicArducopter.BasicArdu.BasicArdu import BasicArdu, Frames

### Global variables and constants ----------
log_file = 'coordinates.txt'

### Main function ----------
def main():

	## Get coordinates.
	drone = BasicArdu( frame = Frames.LLA , connection_string = '/dev/ttyACM0' )
	coords = drone.get_LLA()

	## Print coordinates.
	print( coords )

	## Log coordinates.
	if os.path.isfile( log_file ):
		with open( log_file , 'a+' ) as file:
			file.write( '\n' )
	with open( log_file , 'a+' ) as file:
		ile.write( str( coords ) )

### Program entry point ----------
if __name__ == '__main__':

	main()
