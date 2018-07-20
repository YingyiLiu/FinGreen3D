! ==================================================================================
!   Purpose: This program gives instructions on how to use FinGreen3D.
!
!  License:
! 
!    This routine is part of FinGreen3D.
!
!    FinGreen3D is a free software package: you can redistribute it and/or modify it
!    under the terms of the GNU Lesser General Public License as published by the
!    Free Software Foundation, either version 3 of the License, or (at your option) 
!    any later version.
!
!    You should have received a copy of the GNU General Public License (either V3  
!    or later), along with FinGreen3D. If not, see <http://www.gnu.org/licenses/>.
!
!  Author:
! 
!    Yingyi Liu on Jul.20, 2018
! 
!  Reference:
! 
!    Yingyi Liu et al. A reliable open-source package for performance evaluation of 
!    floating renewable energy systems in coastal and offshore regions. Energy Conversion
!    and management, 2018
!
! ==================================================================================

      Program TutorialProg

        IMPLICIT NONE

        INTEGER  I,J,TAG,NRH
        INTEGER,PARAMETER::NK=200
        REAL*8  R,ZF,ZP,V,H,WVN(NK)
        REAL*8  X,Y,PI,G
        REAL*8,ALLOCATABLE:: RHP(:)
        COMPLEX*16,ALLOCATABLE:: EXCV(:,:),GRN(:,:)

        DATA G,PI /9.807D0,3.141592653589793d0/

        OPEN(1, FILE='InputData.txt',   STATUS='UNKNOWN')
        OPEN(2, FILE='GRN(1).txt',   STATUS='UNKNOWN')
        OPEN(3, FILE='GRN(2).txt',   STATUS='UNKNOWN')
        OPEN(4, FILE='GRN(3).txt',   STATUS='UNKNOWN')

        WRITE(*,*)
        WRITE ( *, '(2x,a)' ) '----------------------------------------------------------------------------------'
        WRITE ( *, '(2x,a)' ) '  Welcome to the use of the software package FinGreen3D.'
        WRITE ( *, '(2x,a)' ) '  This is a tutorial example for teaching you how to call it from a user program. '
        WRITE (*,'(24x,30A,10x)') 'Author: Yingyi Liu, Kyushu University, Jul.20, 2018'
        WRITE ( *, '(2x,a)' ) '----------------------------------------------------------------------------------'

        WRITE (2,'(1x,a6,5(2x,a18))') 'R/H','Re(G)_Newman','Im(G)_Newman','Re(G)_present','Im(G)_present','Relative Error'
        WRITE (3,'(1x,a6,5(2x,a18))') 'R/H','Re(GR)_Newman','Im(GR)_Newman','Re(GR)_present','Im(GR)_present','Relative Error'
        WRITE (4,'(1x,a6,5(2x,a18))') 'R/H','Re(Gz)_Newman','Im(Gz)_Newman','Re(Gz)_present','Im(Gz)_present','Relative Error'

        WRITE(*,*)

!     Read the input file

        READ(1,*)
        READ(1,*) ZF, ZP, V, H
        READ(1,*) TAG, NRH
        PRINT*, 'Echo input parameters:'
        WRITE(*,'(2x,a6,f10.6,2x,a6,f10.6,2x,a6,f10.6,2x,a6,f10.6)')  'ZF=',ZF, 'ZP=',ZP, 'V=',V, 'H=', H
        WRITE(*,'(4x,a6,i2,4x,a6,i4,4x,a16,f10.6,a4)')  'TAG=',TAG, 'NRH=',NRH, 'Wave Frequency=', DSQRT(G*V)/(2.D0*PI),'Hz'

        ALLOCATE(RHP(NRH),EXCV(NRH,3),GRN(NRH,3))
        CALL DISPERSION(WVN,NK,DSQRT(G*V),H)

        DO I=1,12
         READ(1,*)
        ENDDO

        DO J=1,3
         READ(1,*)
         READ(1,*)
         DO I=1,NRH
          READ(1,*) RHP(I),X,Y
          EXCV(I,J)=DCMPLX(X,Y)
         ENDDO
        ENDDO

!     Calculate the Green function by calling FINGREEN3D

        DO I=1,NRH

         R=RHP(I)*H

         CALL FINGREEN3D(R,ZF,ZP,V,WVN,NK,H,GRN(I,:),TAG)

         WRITE(2,1020) R/H,EXCV(I,1),GRN(I,1),CDABS(EXCV(I,1)-GRN(I,1))/CDABS(EXCV(I,1))
         WRITE(3,1020) R/H,EXCV(I,2),GRN(I,2),CDABS(EXCV(I,2)-GRN(I,2))/CDABS(EXCV(I,2))
         WRITE(4,1020) R/H,EXCV(I,3),GRN(I,3),CDABS(EXCV(I,3)-GRN(I,3))/CDABS(EXCV(I,3))

        ENDDO        

        DO J=1,3
         PRINT*
         PRINT*
         IF (J==1) THEN
          WRITE (*,'(1x,a5,5(2x,a14))') 'R/H','Re(G)_Newman','Im(G)_Newman','Re(G)_present','Im(G)_present','Relative Error'
         ELSE IF  (J==2) THEN
          WRITE (*,'(1x,a5,5(2x,a14))') 'R/H','Re(GR)_Newman','Im(GR)_Newman','Re(GR)_present','Im(GR)_present','Relative Error'
         ELSE IF  (J==3) THEN
          WRITE (*,'(1x,a5,5(2x,a14))') 'R/H','Re(Gz)_Newman','Im(Gz)_Newman','Re(Gz)_present','Im(Gz)_present','Relative Error'
         ENDIF
         DO I=1,NRH
          WRITE(*,1050) RHP(I),EXCV(I,J),GRN(I,J),CDABS(EXCV(I,J)-GRN(I,J))/CDABS(EXCV(I,J))
         ENDDO
        ENDDO
        
        DEALLOCATE(RHP,EXCV,GRN)
        
        PRINT*
        PRINT*, 'The test program runs successfully.'
        
1020    FORMAT(F8.4,5E20.12)
1050    FORMAT(2X,F8.4,5E14.4)
        
      END Program TutorialProg      


! ==================================================================================
!   Purpose: This subroutine computes roots of the water-wave dispersion equation 
!             in finite water depth, by using a higher-order iterative method
!
!  License:
! 
!    This routine is part of FinGreen3D.
!
!    FinGreen3D is a free software package: you can redistribute it and/or modify it
!    under the terms of the GNU Lesser General Public License as published by the
!    Free Software Foundation, either version 3 of the License, or (at your option) 
!    any later version.
!
!    You should have received a copy of the GNU General Public License (either V3  
!    or later), along with FinGreen3D. If not, see <http://www.gnu.org/licenses/>.
!
!  Author:
! 
!    Yingyi Liu on Mar.23, 2017
! 
!  Reference:
! 
!    Yingyi Liu et al. A reliable open-source package for performance evaluation of 
!    floating renewable energy systems in coastal and offshore regions. Energy Conversion
!    and management, 2018
! 
!    J.N. Newman
!    Numerical solutions of the water-wave dispersion relation
!    Applied Ocean Research 12 (1990) 14-18
!
!  Parameters:
!      Input:   NRT --- Integer, the number of roots required
!                W   --- Real, wave angular frequency
!                H   --- Real, water depth (h>0)
!      Output:  WVN --- Real, an array storing roots of the dispersion equation
! ==================================================================================
  
    SUBROUTINE DISPERSION(WVN,NRT,W,H)
!
!   Evaluation of the roots of the following equations 
!   by higher-order iterative method
!   first root stored in WVN is from Eq. (i)
!   the rest roots are from Eq. (ii)
!   i) w*w/g = k tanh ( kh )
!   ii) -w*w/g = Um tan ( Umh )
!

      IMPLICIT NONE
      INTEGER,INTENT(IN):: NRT
      REAL*8,INTENT(IN):: W,H
      REAL*8,INTENT(OUT):: WVN(1:NRT)
      INTEGER I, M
      REAL*8 T,X,U,Y,DNM,G,PI
      REAL*8 FUN,DFUN,D2FUN,TRIAL,EXX

      DATA G,PI/9.807d0,3.141592653589793d0/

!------------------------------------------------------------------
! I. calculation of wave number (root of Eq. (i))
!------------------------------------------------------------------
!
!   initialize iteration by an accurate Chebyshev approximation
!   if y=x, use the approximation directly insteady of iteration
!   to avoid the singularity in the denomenator of the transcendental
!   function; otherwise, do the iterative procedure. 
!
      X=W*W*H/G
      IF (X.GT.0.D0.AND.X.LE.2.D0) THEN
       Y=DSQRT(X)*(0.9994D0+0.1701D0*X+0.0305*X*X)
      ELSE
       T=X*DEXP(-2.D0*X)
       Y=X+2.D0*T-6.D0*T*T
      ENDIF

      IF (DABS(Y-X).LT.1.E-10) THEN
       WVN(1)=X/H
      ELSE
       M=0
       EXX=1.D0
       DO WHILE (EXX.GT.1.0D-10)
        TRIAL=Y
        DNM=TRIAL*TRIAL-X*X
        FUN=DLOG((TRIAL+X)/(TRIAL-X))/2.D0-TRIAL
        DFUN=-X/DNM-1.D0
        D2FUN=2.D0*X*TRIAL/(DNM*DNM)
        Y=TRIAL-FUN/DFUN*(1.D0+(FUN/DFUN)*(D2FUN/DFUN)/2.D0)
        EXX=DABS(Y-TRIAL)
        M=M+1
       ENDDO
       WVN(1)=Y/H
      ENDIF

!------------------------------------------------------------------
! II. calcultion of roots of Eq. (ii), which characterizes
!     the evanescene modes in eigenfucntion
!------------------------------------------------------------------
!
!   initialize iteration by a suitable starting approximation
!
      U=3.D0*X/(7.D0+3.D0*X)
      T=0.0159D0+0.1032D0*U+4.3152D0*U*U-2.8768D0*U*U*U
!
!   perform iterative procedure to find exact solution of Um (m=1,..NRT-1)
!   of the transcendental equation Eq. (ii)
!
      DO I=2,NRT
       M=0
       EXX=1.D0
       DO WHILE (EXX.GT.1.0D-10)
        TRIAL=T
        Y=(I-1)*PI-TRIAL
        DNM=Y*Y+X*X
        FUN=ATAN2(X,Y)-TRIAL
        DFUN=X/DNM-1.D0
        D2FUN=2.D0*X*TRIAL/(DNM*DNM)
        T=TRIAL-FUN/DFUN*(1.D0+(FUN/DFUN)*(D2FUN/DFUN)/2.D0)
        EXX=DABS(T-TRIAL)
        M=M+1
       ENDDO
       Y=(I-1)*PI-T
       WVN(I)=Y/H
       T=T-PI*X/(X*X+PI*I*(PI*(I-1)-T))
      ENDDO

      END SUBROUTINE DISPERSION
