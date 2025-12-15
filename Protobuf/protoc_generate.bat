echo -- /protobuf/PackedPolygon.proto --
protoc.exe ../protobuf/PackedPolygon.proto ^
	--csharp_out=../protobuf ^
	--proto_path=../protobuf
call:check_error


echo.
pause
goto:eof


rem changes console text color to red if the last command did not return successful
:check_error
if %errorlevel% NEQ 0 (
	color 04
	echo ^!^! ERROR ^!^!
)
echo.
goto:eof