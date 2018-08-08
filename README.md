
                                                        FINGREEN3D

FINGREEN3D is an open-source package for computation of the free-surface Green's function under a finite water depth, which is the core-part of the boundary integral equations in the potential flow theory for analysis of wave-structure interactions. It is currently written in FORTRAN 90. The code is based on a state-of-the-art method developed in recent years by the authors. The main strategy of this method is to use a set of different series expansions in appropriate sub-regions. Therefore, we can avoid calculating the Green's function by direct time-consuming integrations. 

FINGREEN3D is freely distributed under the LGPL License, Version 3.0, https://www.gnu.org/licenses/lgpl-3.0.html, and may be modified and 
extended by researchers who intend to enhance its capabilities and port the code to other platforms.

It should be noted that, any modified version should be licensed under the LGPL License and be released open-publicly as well. The contributors can add their names in the "contributors list" before the modified subroutine(s).

- Unpacking the package

Copy the FINGREEN3D.tar.gz file to a directory. Unpack the archive by giving the following commands in Linux: gunzip FINGREEN3D_Package.tar.gz and tar -xvf FINGREEN3D_Package.tar. Or you can use the open-source freeware 7-Zip to unpack it in Windows.

- Compiling the package and building the executables

The subfolder 'SourceFiles' contains the source codes, where two options for compilation are provided. In Windows, please run the batch file named 'Makefile-windows.bat' and it will generate an executable file FINGREEN3D.exe for you. In Linux, please run the file 'Makefile' instead, by typing 'make' in the terminal. Pay attention that the open-public tool gfortran should be available in the system beforehand.

- Running the tutorial examples

For instruction purpose, there exist two examples under the unpacked subfolder 'TestExamples'. Please copy the executable file FINGREEN3D.exe (for Windows) or FINGREEN3D.out (for Linux) to the subfolder 'Test01' or 'Test02', and then run it. Results by the authors are provided as well, for you to check if your implementation is successful or not.

- Using the library in other applications

There are two ways of using FINGREEN3D. You can either link it as an external dynamic-link library with your frequency-domain potential flow solver, or simply use it to compute values of free-surface Green's function under some prescribed conditions. No matter what you do, the calling style is the same and also extremely simple, just by typing:

CALL FINGREEN3D(R,ZF,ZP,V,WVN,NK,H,GRN,TAG)

Note that the input & output parameters should keep the correct form as described in the accompanying ECM paper:

Yingyi Liu et al. A reliable open-source package for performance evaluation of floating renewable energy systems in coastal and offshore regions. Energy Conversion and management, 2018.

Please cite the above paper in your relevant publications if the FINGREEN3D code or its executable (dynamic link library) has contributed to your work.

NO WARRANTY is given to FINGREEN3D.

The authors are not liable for any misuse or real and/or presumed damage which can arise from an improper use or removing/deleting some 
parts of this software.
