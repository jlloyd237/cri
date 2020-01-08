MODULE ABBServer
	!// ABB RAPID server for Common Robot Interface
	!// Adapted from Open ABB RAPID server https://github.com/robotics/open_abb

	!// Robot configuration
	PERS tooldata currentTool := [TRUE, [[0,0,0], [1,0,0,0]], [0.001, [0,0,0.001], [1,0,0,0], 0, 0, 0]];    
	PERS wobjdata currentWobj := [FALSE, TRUE, "", [[0,0,0], [1,0,0,0]], [[0,0,0], [1,0,0,0]]];   
	PERS speeddata currentSpeed;
	PERS zonedata currentZone;	
	VAR extjoint externalAxis;

	!// Bounding box for safe workspace
	VAR wzstationary box;
	VAR wzstationary box2;
	VAR wzstationary cyl;

	!// Communications
	PERS string serverIp := "127.0.0.1";
	PERS num serverPort := 5000;
	VAR socketdev clientSocket;
	VAR socketdev serverSocket;

	!// Set up bounding box for safe workspace	
	PROC InitializeBoundingBox()
		CONST pos box_c1 := [-600,-500,30];
		CONST pos box_c2 := [600,500,500];
		CONST pos box2_c1 := [-115,-115,-115];
		CONST pos box2_c2 := [115,115,115];
		CONST pos cyl_p := [0,0,0];
		CONST num cyl_r := 200;
		CONST num cyl_h := 500;

		VAR shapedata volume;

		WZBoxDef \Outside, volume, box_c1, box_c2;
		WZLimSup \Stat, box, volume;
		WZBoxDef \Inside, volume, box2_c1, box2_c2;
		WZLimSup \Stat, box2, volume;
		WZCylDef \Inside, volume, cyl_p, cyl_r, cyl_h;
		WZLimSup \Stat, cyl, volume;	
	ENDPROC

	!// Initialize robot configuration
	PROC InitializeRobotConfig()
		VAR jointtarget jointAngles;
		
		currentTool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];    
		currentWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
		currentSpeed := [100, 50, 0, 0];
		currentZone := [TRUE, 0.3, 0.3,0.3,0.03,0.3,0.03]; !z0		
		
		!// Find the current external axis values so they don't move when we start
		jointAngles := CJointT();
		externalAxis := jointAngles.extax;
	ENDPROC

	!// Create socket and wait for incoming connections
	PROC InitializeSocket(string ip, num port)
		VAR string clientIP;
		
		SocketCreate serverSocket;
		SocketBind serverSocket, ip, port;
		SocketListen serverSocket;
		TPWrite "SERVER: Waiting for incoming connections ...";
		WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
			SocketAccept serverSocket, clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
			IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
				TPWrite "SERVER: Error accepting incoming connection";
				TPWrite "SERVER: Try reconnecting";
			ENDIF
			!// Wait 0.5 seconds for the next reconnection
			WaitTime 0.5;
		ENDWHILE
		TPWrite "SERVER: Connected to IP address " + clientIP;
	ENDPROC
	
	PROC power_on()
		InitializeBoundingBox;
	ENDPROC

	PROC main()
		CONST num SERVER_ERROR :=  0;
		CONST num SERVER_OK := 1;
		
		VAR rawbytes receiveMsg;
		VAR rawbytes sendMsg;	
		
		VAR bool connected;			!// Client connected
		VAR bool reconnected;		!// Connection dropped and reconnected while servicing a command
		
		VAR num command;
		VAR num params{14};
		VAR num ack;		
		VAR string info;

		VAR jointtarget jointAngles;
		VAR robtarget pose;
		VAR robtarget viaPose;
		VAR robtarget endPose;		
					
		!// Configure motion
		ConfL \Off;					!// Don't monitor robot config during linear movement
		!// SingArea \Wrist;
		
		!// Initialize robot configuration
		InitializeRobotConfig;

		!// Initialize socket connection
		connected := FALSE;
	    serverIp := GetSysInfo(\LanIp);
		IF serverIp = "VC" THEN
			serverIp := "127.0.0.1";
		ENDIF
		InitializeSocket serverIp, serverPort;	
		connected := TRUE;
		
		!// Main server loop ...
		WHILE TRUE DO
			ack := SERVER_OK;
			reconnected := FALSE;

			!// Wait for incoming request from client
			SocketReceive clientSocket \RawData:=receiveMsg \Time:=WAIT_MAX;

			!// Read command
			UnpackRawBytes receiveMsg \Network, 1, command \IntX:=UINT;
			
			!// Process command
			TEST command
				CASE 0:		!// Get info
					!// Perform command					
					info := GetSysInfo(\SerialNo) + ", ";
					info := info + GetSysInfo(\SWVersion) + ", ";
					info := info + GetSysInfo(\RobotType) + ", ";
					info := info + GetSysInfo(\LanIp);

					!// Compile response
					ClearRawBytes sendMsg;
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;
					PackRawBytes info, sendMsg \Network, RawBytesLen(sendMsg)+1 \ASCII;
				
				CASE 1:		!// Move joints
					!// Get joint angles
					UnpackRawBytes receiveMsg \Network, 3, params{1} \Float4;
					UnpackRawBytes receiveMsg \Network, 7, params{2} \Float4;
					UnpackRawBytes receiveMsg \Network, 11, params{3} \Float4;
					UnpackRawBytes receiveMsg \Network, 15, params{4} \Float4;
					UnpackRawBytes receiveMsg \Network, 19, params{5} \Float4;
					UnpackRawBytes receiveMsg \Network, 23, params{6} \Float4;					
					jointAngles := [[params{1}, params{2}, params{3},
									params{4}, params{5}, params{6}], externalAxis];
					
					!// Perform command
					MoveAbsJ jointAngles, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
					
					!// Compile response
					ClearRawBytes sendMsg;					
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;

				CASE 2:		!// Move linear
					!// Read pose
					UnpackRawBytes receiveMsg \Network, 3, params{1} \Float4;
					UnpackRawBytes receiveMsg \Network, 7, params{2} \Float4;
					UnpackRawBytes receiveMsg \Network, 11, params{3} \Float4;
					UnpackRawBytes receiveMsg \Network, 15, params{4} \Float4;
					UnpackRawBytes receiveMsg \Network, 19, params{5} \Float4;
					UnpackRawBytes receiveMsg \Network, 23, params{6} \Float4;
					UnpackRawBytes receiveMsg \Network, 27, params{7} \Float4;					
					pose := [[params{1}, params{2}, params{3}],
							[params{4}, params{5}, params{6}, params{7}],
							[0,0,0,0],
							externalAxis];
									   
					!// Perform command			   
					MoveL pose, currentSpeed, currentZone, currentTool \WObj:=currentWobj;

					!// Compile response
					ClearRawBytes sendMsg;					
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;
				
				CASE 3:		!// Move circular
					!// Read via pose
					UnpackRawBytes receiveMsg \Network, 3, params{1} \Float4;
					UnpackRawBytes receiveMsg \Network, 7, params{2} \Float4;
					UnpackRawBytes receiveMsg \Network, 11, params{3} \Float4;
					UnpackRawBytes receiveMsg \Network, 15, params{4} \Float4;
					UnpackRawBytes receiveMsg \Network, 19, params{5} \Float4;
					UnpackRawBytes receiveMsg \Network, 23, params{6} \Float4;
					UnpackRawBytes receiveMsg \Network, 27, params{7} \Float4;
					viaPose := [[params{1}, params{2}, params{3}],
								[params{4}, params{5}, params{6}, params{7}],
								[0,0,0,0],
								externalAxis];
								
					!// Read end pose
					UnpackRawBytes receiveMsg \Network, 31, params{8} \Float4;
					UnpackRawBytes receiveMsg \Network, 35, params{9} \Float4;
					UnpackRawBytes receiveMsg \Network, 39, params{10} \Float4;
					UnpackRawBytes receiveMsg \Network, 43, params{11} \Float4;
					UnpackRawBytes receiveMsg \Network, 47, params{12} \Float4;
					UnpackRawBytes receiveMsg \Network, 51, params{13} \Float4;
					UnpackRawBytes receiveMsg \Network, 55, params{14} \Float4;
					endPose := [[params{8}, params{9}, params{10}],
								[params{11}, params{12}, params{13}, params{14}],
								[0,0,0,0],
								externalAxis];
										
					!// Perform command					
					MoveC viaPose, endPose, currentSpeed, currentZone, currentTool \WObj:=currentWobj;

					!// Compile response	
					ClearRawBytes sendMsg;					
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;
					
				CASE 4:		!// Set tcp
					!// Read tcp
					UnpackRawBytes receiveMsg \Network, 3, params{1} \Float4;
					UnpackRawBytes receiveMsg \Network, 7, params{2} \Float4;
					UnpackRawBytes receiveMsg \Network, 11, params{3} \Float4;
					UnpackRawBytes receiveMsg \Network, 15, params{4} \Float4;
					UnpackRawBytes receiveMsg \Network, 19, params{5} \Float4;
					UnpackRawBytes receiveMsg \Network, 23, params{6} \Float4;
					UnpackRawBytes receiveMsg \Network, 27, params{7} \Float4;				

					!// Perform command					
					currentTool.tframe.trans.x := params{1};
					currentTool.tframe.trans.y := params{2};
					currentTool.tframe.trans.z := params{3};
					currentTool.tframe.rot.q1 := params{4};
					currentTool.tframe.rot.q2 := params{5};
					currentTool.tframe.rot.q3 := params{6};
					currentTool.tframe.rot.q4 := params{7};

					!// Compile response
					ClearRawBytes sendMsg;			
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;
				
				CASE 5:		!// Set work object
					!// Read work object
					UnpackRawBytes receiveMsg \Network, 3, params{1} \Float4;
					UnpackRawBytes receiveMsg \Network, 7, params{2} \Float4;
					UnpackRawBytes receiveMsg \Network, 11, params{3} \Float4;
					UnpackRawBytes receiveMsg \Network, 15, params{4} \Float4;
					UnpackRawBytes receiveMsg \Network, 19, params{5} \Float4;
					UnpackRawBytes receiveMsg \Network, 23, params{6} \Float4;	
					UnpackRawBytes receiveMsg \Network, 27, params{7} \Float4;

					!// Perform command					
					currentWobj.oframe.trans.x := params{1};
					currentWobj.oframe.trans.y := params{2};
					currentWobj.oframe.trans.z := params{3};
					currentWobj.oframe.rot.q1 := params{4};
					currentWobj.oframe.rot.q2 := params{5};
					currentWobj.oframe.rot.q3 := params{6};
					currentWobj.oframe.rot.q4 := params{7};

					!// Compile response
					ClearRawBytes sendMsg;
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;					

				CASE 6:		!// Set speed
					!// Read speed
					UnpackRawBytes receiveMsg \Network, 3, params{1} \Float4;
					UnpackRawBytes receiveMsg \Network, 7, params{2} \Float4;				

					!// Perform command					
					currentSpeed.v_tcp := params{1};
					currentSpeed.v_ori := params{2};

					!// Compile response
					ClearRawBytes sendMsg;					
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;

				CASE 7:		!// Set zone
					!// Read zone data
					UnpackRawBytes receiveMsg \Network, 3, params{1} \IntX:=UINT;
					UnpackRawBytes receiveMsg \Network, 5, params{2} \Float4;
					UnpackRawBytes receiveMsg \Network, 9, params{3} \Float4;
					UnpackRawBytes receiveMsg \Network, 13, params{4} \Float4;

					!// Perform command					
					IF params{1}=1 THEN
						currentZone.finep := TRUE;
						currentZone.pzone_tcp := 0.0;
						currentZone.pzone_ori := 0.0;
						currentZone.zone_ori := 0.0;
					ELSE
						currentZone.finep := FALSE;
						currentZone.pzone_tcp := params{2};
						currentZone.pzone_ori := params{3};
						currentZone.zone_ori := params{4};
					ENDIF

					!// Compile response
					ClearRawBytes sendMsg;	
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;
					
				CASE 8:		!// Get joint angles
					!// Perform command	
					jointAngles := CJointT();

					!// Compile response
					ClearRawBytes sendMsg;
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;
					PackRawBytes jointAngles.robax.rax_1, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes jointAngles.robax.rax_2, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes jointAngles.robax.rax_3, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes jointAngles.robax.rax_4, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes jointAngles.robax.rax_5, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes jointAngles.robax.rax_6, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;				

				CASE 9:		!// Get pose
					!// Perform command	
					pose := CRobT(\Tool:=currentTool \WObj:=currentWObj);

					!// Compile response
					ClearRawBytes sendMsg;
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;
					PackRawBytes pose.trans.x, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes pose.trans.y, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes pose.trans.z, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes pose.rot.q1, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes pose.rot.q2, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes pose.rot.q3, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;
					PackRawBytes pose.rot.q4, sendMsg \Network, RawBytesLen(sendMsg)+1 \Float4;					

				CASE 99:	!// Close connection
					TPWrite "SERVER: Client has closed connection";
					connected := FALSE;
					!// Close server
					SocketClose clientSocket;
					SocketClose serverSocket;

					!// Restart server
					InitializeSocket serverIp, serverPort;
					connected := TRUE;
					reconnected := TRUE;
				
				DEFAULT:
					TPWrite "SERVER: Invalid command";
					ack := SERVER_ERROR;
					
					!// Compile response
					ClearRawBytes sendMsg;
					PackRawBytes ack, sendMsg \Network, RawBytesLen(sendMsg)+1 \IntX:=UINT;
			ENDTEST
			
			!// Send response to client		
			IF connected = TRUE THEN
				IF reconnected = FALSE THEN
					IF SocketGetStatus(clientSocket) = SOCKET_CONNECTED THEN
						SocketSend clientSocket \RawData:=sendMsg;
					ENDIF
				ENDIF
			ENDIF			
		ENDWHILE

	ERROR (LONG_JMP_ALL_ERR)
		TPWrite "SERVER: ------";
		TPWrite "SERVER: Error Handler:" + NumtoStr(ERRNO, 0);
		TEST ERRNO
			CASE ERR_SOCK_CLOSED:
				TPWrite "SERVER: Lost connection to client";
				TPWrite "SERVER: Shutting down server and restarting ...";
				connected := FALSE;
				!// Shut down server
				SocketClose clientSocket;
				SocketClose serverSocket;
				!// Restart server
				InitializeSocket serverIp, serverPort;
				reconnected := FALSE;
				connected := TRUE;
				TPWrite "SERVER: Server restarted";
				TPWrite "SERVER: ------";				
				RETRY; 
			DEFAULT:
				TPWrite "SERVER: Unknown error";
				TPWrite "SERVER: Shutting down server and restarting ...";
				connected := FALSE;
				!// Shut down server
				SocketClose clientSocket;
				SocketClose serverSocket;
				!// Restart server
				InitializeSocket serverIp, serverPort;
				reconnected := FALSE;
				connected := TRUE;
				TPWrite "SERVER: Server restarted";
				TPWrite "SERVER: ------";
				RETRY;
		ENDTEST
	ENDPROC
ENDMODULE
