function writeOutData_25(obj,event,hObject,handles)
global DataArr

%txmsg='getData'; %Currently, if OMAP receives anything, it transmits the same set of floats
%fprintf('Querying DSP for data...\n');
txmsg=uint8([253 handles.TCPIP.txmsg 255]); %string 'GetData' is arbitrary for now.
%Conceivably, we can set up DSP to send back different things based on transmitted strings
handles.TCPIP.out_stream.write(txmsg);



tic
while(toc<0.5 && handles.TCPIP.in_stream.available<1)
    %wait for either 0.5 seconds to pass or the input stream to show
    %available bytes
    %fprintf('waiting for more bytes on stream \n')
end
% toc

if(handles.TCPIP.in_stream.available()<1)
    fprintf('No bytes to read in input stream. Stopping timer object \n');
    
    %stop the timer just like i do in disconnect button callback 
    %(calling disconnect callback from here wont let me pass the handles structure.
    %From this function's perspective, the 'disconnect' callback function 
    %takes only two arguments, not third argument for 'handles'.)
    stop(handles.TCPIP.writeOutTimer)
    fprintf('Timer Object Stopped\n')
    handles.TCPIP.in_stream.close();
    handles.TCPIP.out_stream.close();
    handles.TCPIP.mySocket.close();
    handles.TCPIP.mySocket = [];
    set(handles.tb_TickerTxt,'String','Conn Closed')
    set(handles.tb_TickerTxt,'BackgroundColor',[0.941 0.941 0.941])
    fprintf('Connection successfully closed\n')
    fprintf('Socket closed\nRESTART VBDSPCOMM.\n')
else
    handles.TCPIP.in_stream.read(); %get rid of start character
    numchars_str=[];
    
    tempstr=[];
    numchars_str=[];
    while (isempty(tempstr)||char(tempstr)~=',') %order of this while condition is important, because " []~=',' " evaluates to "[]"
        numchars_str=[numchars_str,tempstr];
        tempstr=handles.TCPIP.in_stream.read();
    end
    %fprintf('First comma has been read into the buffer... \n');
numchars=str2num(char(numchars_str)); % gets number of remaining characters in message
%fprintf('number of data characters:  %d \n',numchars);
rxBytes=handles.TCPIP.reader.read(handles.TCPIP.mySocket,numchars);

tempstr=handles.TCPIP.in_stream.read(); %read off the end character
%fprintf('Read off end char: %d\n',tempstr);

out1=handles.TCPIP.reader.bfr;

scanString=[];
for i=1:handles.TCPIP.numFloats-1
    scanString=[scanString '%f,'];
end
scanString=[scanString,'%f'];

[DataArr]=sscanf(char(out1(1:numchars)),scanString);


str = [];
for i=1:17
    b='0000000000000';
    str = [str; b];
    a=dec2bin(DataArr(i));
    for j=1:length(a)
        str(i,13-length(a)+j)=a(j);
    end
end

%convert array of chars to array of ints
for i=1:17 
    for j=1:13
       str2(i,j)=str2num(str(i,j));
    end
end

% DataArr(22:25) = [];

% if(DataArr(22:25) == [0 0 0 0])
%     set(handles.Plotting.OBalls,'Marker',none,'MarkerSize',10,'MarkerFaceColor',none,'XData',DataArr(24),'YData',DataArr(25),'LineStyle','none');
%     set(handles.Plotting.BBalls,'Marker',none,'MarkerSize',10,'MarkerFaceColor',none,'XData',DataArr(22),'YData',DataArr(23),'LineStyle','none');
% else
%     set(handles.Plotting.OBalls,'Marker','o','MarkerSize',10,'MarkerFaceColor','y','XData',DataArr(24),'YData',DataArr(25),'LineStyle','none');
%     set(handles.Plotting.BBalls,'Marker','o','MarkerSize',10,'MarkerFaceColor','c','XData',DataArr(22),'YData',DataArr(23),'LineStyle','none');
% end

%Robot path
% set(handles.myPlot,'XData',DataArr(40:2:(40 + (DataArr(20)-1))),'YData',DataArr(41:2:(40 + (DataArr(20) - 1))));

%set(handles.TCPIP.DataArr,DataArr)
%set(handles.TCPIP.okToWrite,0)
%set(handles.TCPIP.okToRead,1)
set(handles.myMaze,'cdata',str2);
set(handles.axes1,'Xlim',[-7,7]);
set(handles.axes1,'Ylim',[-8,13]);
set(handles.myPlot,'Marker','o','MarkerSize',15,'MarkerEdgeColor','k','MarkerFaceColor','r','XData',DataArr(18),'YData',DataArr(19));

Bballsx = [];
Bballsy = [];
Oballsx = [];
Oballsy = [];

for i = 22:3:34
    if (DataArr(i+2) == 1)
        Oballsx = [Oballsx DataArr(i)];
        Oballsy = [Oballsy DataArr(i+1)];
        set(handles.Plotting.OBalls,'Marker','o','MarkerSize',10,'MarkerFaceColor','y','XData',Oballsx,'YData',Oballsy,'LineStyle','none');
    else
        Bballsx = [Bballsx DataArr(i)];
        Bballsy = [Bballsy DataArr(i+1)];
        set(handles.Plotting.BBalls,'Marker','o','MarkerSize',10,'MarkerFaceColor','c','XData',Bballsx,'YData',Bballsy,'LineStyle','none');
    end
end

%set(handles.Plotting.OBalls,'Marker','o','MarkerSize',10,'MarkerFaceColor',[255 165 0],'XData',DataArr(24),'YData',DataArr(25),'LineStyle','none');
% set(handles.Plotting.OBallsloc,'Marker','o','MarkerSize',30,'MarkerFaceColor',[255 165 0],'XData',[-5],'YData',[-2],'LineStyle','none');
% set(handles.Plotting.BBallsloc,'Marker','o','MarkerSize',30,'MarkerFaceColor','c','XData',[-5],'YData',[-2],'LineStyle','none');

% set(handles.Plotting.BBalls,'Marker','o','MarkerSize',10,'MarkerFaceColor','c','XData',[DataArr(25),DataArr(29)],'YData',[DataArr(26),DataArr(30)],'LineStyle','none');
% set(handles.Plotting.OBalls,'Marker','o','MarkerSize',10,'MarkerFaceColor','y','XData',[DataArr(27),DataArr(31)],'YData',[DataArr(28),DataArr(32)],'LineStyle','none');

%set(handles.axes1,'Zlim',[-10,10]);
% for i = 21:29
% if(DataArr(22:29) == [-8 -8 -8 -8 -8 -8 -8 -8])
%     set(handles.tbl_ball_pos,'Data',{[1],[1],[1],[1];[],[],[],[];[],[],[],[];[],[],[],[];[],[],[],[];[],[],[],[]});
% else
set(handles.tbl_ball_pos,'Data',{DataArr(22),DataArr(23),DataArr(24);DataArr(25),DataArr(26),DataArr(27);DataArr(28),DataArr(29),DataArr(30);DataArr(31),DataArr(32),DataArr(33);DataArr(34),DataArr(35),DataArr(36)});
%   end
% end

%set(handles.tbl_ball_pos,'Data',{DataArr(25),DataArr(26),DataArr(27),DataArr(28)});
%guidata(hObject,handles)
set(handles.txt_X,'string',DataArr(18))
set(handles.txt_Y,'string',DataArr(19))
set(handles.txt_Theta,'string',DataArr(20))

end
%guidata(hObject,handles)
end


