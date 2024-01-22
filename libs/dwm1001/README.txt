=============================================================================
README.txt - Decawave's DWM1001 Host Application Programming Interface (API)
=============================================================================

The DWM1001 Host API package is composed of the following folders:

    - dwm_driver:
    
      Driver for DWM1001 module host API. Details about each function can
      be found in DWM1001 API Guide.

    - examples:
    
      A set of individual (simple) examples showing how to achieve different
      functionalities, e.g. setting up an Anchor, setting up an Tag.  The 
      emphasis of theses examples is to be readable with explanatory notes to
      explain and show how the main features of the DWM1001 work and can be 
      used.

    - platform/rpi/hal:
    
      Hardware abstraction layer (system start-up code and peripheral
      drivers) for Raspberry Pi 3 platform as the host device.

    - include:
    
      The files users need to include in their designs: 
      dwm_api.h file must be included in the source file so that the users 
      can use the dwm1001 API functions.
      dwm1001.mak can be include in the makefile for resource reference.
      
Please refer to DWM1001 API Guide accompanying this package for more details
about provided API and examples.

NOTE: The DWM1001 API/driver code included in this package is an unbundled
      version of the DWM1001 API/driver. This version may be different to
      (generally by being newer than) those bundled with Decawave's other
      products.

=============================================================================
=                                                                           =
=                               Release Notes                               =
=                                                                           =
=============================================================================

=============================================================================
Package v0.0.1 / Driver v0.0.1  (14th August 2017)
=============================================================================
