function varargout = Matlab_SendFloats_25(varargin)
% Matlab_SendFloats_25 MATLAB code for Matlab_SendFloats_25.fig
%
%      GUI used to send floats to DSP from Matlab
%
%      General pointers on using MATLAB GUI:
%           If you want to edit the GUI, open the .fig file with the same
%           name as the .m file in the Graphical User Interface Development
%           Environment with the command guide('filename'). Save the file
%           from GUIDE and it will automatically add new callback functions
%           to any existing code you have.
%
%           Each callback function always has 3 arguments: hObject,
%           eventdata, and handles. hObject contains the handle to the
%           calling object (the handle to the pushbutton for pushbutton
%           callback etc.) eventdata contains information about what time
%           the callback occured, and handles is a struct containing
%           handles to all the objects in the ui.
%
%           In practice, you use the handles structure to execute get/set
%           commands on the objects in the figure. This means you are
%           querying their properties (with 'get') or changing their
%           properties (with 'set'). You can also add arbitrary
%           information to the handles structure. For example you can add a
%           field 'myNum' with the statement 'handles.myNum=10;'
%
%           At the end of some functions you see the statement:
%           "guidata(hObject, handles)." This ensures that any changes made
%           to the handles structure during this callback persist after the
%           callback function exits.
%
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Matlab_SendFloats_25

% Last Modified by GUIDE v2.5 06-May-2014 23:31:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Matlab_SendFloats_25_OpeningFcn, ...
    'gui_OutputFcn',  @Matlab_SendFloats_25_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

%notes 3/24/14

%I am able to modify any variables in the original gui using the timer
%function provided I am careful about a few things:
%   1) I specify input arguments in the following way:
%     "set(handles.TCPIP.writeOutTimer,'TimerFcn',{'writeOutData',hObject,handles},...)"
%     Note that this causes the value of hObject (the calling object to the function
%     that created the timer) and handles(the struct containing all objects
%     and data about the GUI) to be accessible in timer function.
%
%    2) If the handles struct is changed AFTER the line "set(handles.TCPIP.writeOutTimer,'TimerFcn',{'writeOutData',hObject,handles},...)"
%       is called, then the changes will not be seen in the timer function.
%       To make the changes available in the timer function, the best way
%       is to stop the timer ('stop(timerObjName)'), rerun the command
%       "set(handles.TCPIP.writeOutTimer,'TimerFcn',{'writeOutData',hObject,handles},...)"
%       and then restart the timer object. This will update the handles
%       array that is visible within the timer function
%
%    3) If you bump timer period to 0.01
%       seconds and try running the connection, nothing is plotted. Also,
%       MATLAB becomes less robust to completely hanging. I'm guessing that if
%       MATLAB can't do anything before it is interrupted by another timer
%       function it becomes useless. For now, 0.1 period is sufficient for
%       plotting purposes.
%
%    * Note that there are no such thing as pointers in MATLAB. The closest
%    thing is what they call "handles" to objects. These handles can be
%    used like pointers, i.e. passed by value into functions and used to
%    edit the objects they point to, but if the object changes, the same
%    handle may not be able to read/write to the modified object fields.

% --- Executes just before Matlab_SendFloats_25 is made visible.
function Matlab_SendFloats_25_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Matlab_SendFloats_25 (see VARARGIN)

global DataArr;
DataArr=[];

handles.TCPIP.numFloats=25; %change if you want to send more floats
handles.TCPIP.mySocket = [];
handles.TCPIP.out_stream = [];
handles.TCPIP.in_stream = [];
handles.TCPIP.txmsg = 'GetData';
handles.UserData=[];

hold on
handles.myMaze=imagesc([6,-6],[12,-4],zeros(17,13));
% handles.myMaze=(zeros(17,13));
handles.myPlot=plot(handles.axes1,[-1]); %we initialize the plot to have some data so it is given a handle in our handles structure
set(handles.myMaze,'parent',handles.axes1)
% set(handles.axes1,'Xlim',[-8,8]);
% set(handles.axes1,'Ylim',[-4,13]);
% set(handles.axes1,'Zlim',[-10,10]);
set(handles.figure1,'toolbar','figure');
%set(handles.figure1,'menubar','figure');

%right now our timer only asks for 10 floats, if more information needs to
%be sent, consider adding to the 10 floats (or adding a second write out
%timer)
%Also, we don't give the timers a function yet. This is because our handles
%structure doesn't yet contain in_stream and out_stream objects. Once we
%declare input arguments for the timer, those arguments are constants
%unless we redeclare the timer input arguments with a "set" statement.

handles.TCPIP.writeOutTimer=timer('ErrorFcn','disp(''Timer object error'')','ExecutionMode','fixedRate');

% Choose default command line output for Matlab_SendFloats_25
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Matlab_SendFloats_25 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = Matlab_SendFloats_25_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function edt_RobotIP_Callback(hObject, eventdata, handles)
% hObject    handle to edt_RobotIP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes during object creation, after setting all properties.

function edt_RobotIP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edt_RobotIP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edt_Port_Callback(hObject, eventdata, handles)
% hObject    handle to edt_Port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function edt_Port_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edt_Port (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pb_Connect.
function pb_Connect_Callback(hObject, eventdata, handles)
% hObject    handle to pb_Connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%set ticker text to blue while it tries to connect
set(handles.tb_TickerTxt,'String','Connecting...')
set(handles.tb_TickerTxt,'BackgroundColor',[0.3 0.3 1])

%check to see if Socket object already exists. If it does, then we do
%nothing.
if(~isempty(handles.TCPIP.mySocket))
    fprintf('Socket already exists.\n')
    set(handles.tb_TickerTxt,'String','Connected')
    set(handles.tb_TickerTxt,'BackgroundColor',[0.3 1 0.3])
else
    %No socket has been opened yet
    port=str2num(get(handles.edt_Port,'String'));
    host=get(handles.edt_RobotIP,'String');
    import java.net.Socket
    import java.io.*
    javaaddpath('.') % we add the current directory to our java path so we can make an instance of our custom class
    
    try
        handles.TCPIP.reader=InputStreamByteWrapper;
        wrapperCreated=true;
    catch
        %fprintf('Socket was opened, but Java Wrapper class not recognized.\n');
        %fprintf('Socket will be closed...\n\n');
        %PBcloseSocket_Callback(hObject, eventdata, handles)
        fprintf('Error:Does your current directory contain InputStreamByteWrapper.class file?.\n');
        set(handles.tb_TickerTxt,'String','Error')
        set(handles.tb_TickerTxt,'BackgroundColor',[1 0.3 0.3])
        wrapperCreated=false;
    end
    
    %if we successfully made wrapper class, we try to open a socket
    if(wrapperCreated)
        try
            handles.TCPIP.mySocket = Socket(host, port);
            handles.TCPIP.in_stream  = handles.TCPIP.mySocket.getInputStream;
            handles.TCPIP.out_stream = handles.TCPIP.mySocket.getOutputStream;
            fprintf('New socket opened.\nHost: %s, Port: %d \n',host,port);
            %Initialize timers here because all the subfields which we will
            %refer to in the timer function now exist
            set(handles.TCPIP.writeOutTimer,'TimerFcn',{'writeOutData_25',hObject,handles},'Period',0.1)
            fprintf('Timer reinitialized\n');
            set(handles.tb_TickerTxt,'String','Connected');
            set(handles.tb_TickerTxt,'BackgroundColor',[0.3 1 0.3])
            start(handles.TCPIP.writeOutTimer);
        catch
            clear reader; %get rid of java object
            handles.TCPIP.mySocket=[]; %We just don't want 'clear java' command to leave TCPIP.mySocket undefined.
            %Currently I use handles.TCPIP.mySocket=[] to mean that we will
            %need to create a new socket object when the user wishes to
            %connect, and not reuse an old socket. In this way, the
            %behavior of the connect button is the same regardless of if
            %the user just opened the application or has just closed his
            %last socket.
            fprintf('Error: Socket not opened successfully.\n')
            fprintf('Check host ip, and socket number.\n')
            fprintf('Also, make sure robot is running VBDSPComm.\n')
            set(handles.tb_TickerTxt,'String','Error')
            set(handles.tb_TickerTxt,'BackgroundColor',[1 0.3 0.3])
        end
    end
    guidata(hObject,handles);
end


% --- Executes on button press in pb_Disconnect.
function pb_Disconnect_Callback(hObject, eventdata, handles)
% hObject    handle to pb_Disconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(~isempty(handles.TCPIP.mySocket))
    stop(handles.TCPIP.writeOutTimer)
    fprintf('Timer Object Stopped\n')
    handles.TCPIP.in_stream.close();
    handles.TCPIP.out_stream.close();
    handles.TCPIP.mySocket.close();
    handles.TCPIP.mySocket = [];
    set(handles.tb_TickerTxt,'String','Conn Closed')
    set(handles.tb_TickerTxt,'BackgroundColor',[0.941 0.941 0.941])
    fprintf('Connection successfully closed\n')
    fprintf('Socket closed\nRestart VBDSPComm before connecting again.\n')
    guidata(hObject,handles)
else
    set(handles.tb_TickerTxt,'String','Error')
    set(handles.tb_TickerTxt,'BackgroundColor',[1 0.3 0.3])
    fprintf('Socket was already closed \n')
end

% --- Executes on button press in tb_TickerTxt.
function tb_TickerTxt_Callback(hObject, eventdata, handles)
% hObject    handle to tb_TickerTxt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%just using this assignment to place breakpoints next to...
fprintf('Why did you press the ticker? \n');

%stop timer, update transmitted function, restart timer (beta testing)
if(~isempty(handles.TCPIP.mySocket))
    stop(handles.TCPIP.writeOutTimer)
    handles.TCPIP.txmsg=num2str(rand());
    set(handles.TCPIP.writeOutTimer,'TimerFcn',{'writeOutData_25',hObject,handles})
    start(handles.TCPIP.writeOutTimer)
end

% --- Executes on button press in pb_X.
function pb_X_Callback(hObject, eventdata, handles)
% hObject    handle to pb_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pb_Y.
function pb_Y_Callback(hObject, eventdata, handles)
% hObject    handle to pb_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in pb_Theta.
function pb_Theta_Callback(hObject, eventdata, handles)
% hObject    handle to pb_Theta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    pb_Disconnect_Callback(hObject, eventdata, handles)
catch
    fprintf('Disconnect attempt failed. This is normal if you just opened the .fig file without GUIDE.')
end
try
    %stop(handles.TCPIP.readInTimer)
    stop(handles.TCPIP.writeOutTimer)
    %delete(handles.TCPIP.readInTimer)
    delete(timerfindall)
    fprintf('Timer objects successfully deleted\n')
catch
    fprintf('something wrong with deletion of timers\n')
end
% Hint: delete(hObject) closes the figure
delete(hObject);



function edt_SendVals_Callback(hObject, eventdata, handles)
% hObject    handle to edt_SendVals (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edt_SendVals as text
%        str2double(get(hObject,'String')) returns contents of edt_SendVals as a double


% --- Executes during object creation, after setting all properties.
function edt_SendVals_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edt_SendVals (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_SendVals.
function pb_SendVals_Callback(hObject, eventdata, handles)
% hObject    handle to pb_SendVals (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
