rem yes this is hardcoded but this is scratchpad. if you want to make this work...
rem just put the peel repo in the parent directory of scratchpad repo
robocopy AbominationInterop\bin\Release\net7.0\win10-x64\native\ ..\..\..\peel\PEEL_PhysX_Edition\PEEL_Externals\Bepu\Release
robocopy AbominationInterop\bin\Debug\net7.0\win10-x64\native\ ..\..\..\peel\PEEL_PhysX_Edition\PEEL_Externals\Bepu\Debug
robocopy BepuPhysicsCPP ..\..\..\peel\PEEL_PhysX_Edition\PEEL_Externals\Bepu *.h