README.txt

 Created on: 8th  May 2012
 Updated on: 31st Jan 2013
 Updated on: 16th Jun 2015 (to work with v3.3 and up to r24460)
 Updated on: 16th Feb 2016 (to work with v3.4 and up to r26040)
 
     Author: Gary Mirams
     
This project contains all the code required to recreate the results in the PloS 2013 paper.

It must be checked out into

<chaste directory>/projects/Plos2013 

in order for file paths to be picked up correctly, and it should be used with Chaste v3.4

To run the examples you should do (for example):
cd <chaste directory>
scons build=GccOpt test_suite=projects/Plos2013/test/TestCryptsAndVillusLiteratePaper.hpp

There is a Chaste wiki page associated with this project that gives links to more information on Chaste installation,
and the example simulations that are in the 'test' folder:
https://chaste.cs.ox.ac.uk/cgi-bin/trac.cgi/wiki/PaperTutorials/Plos2013
