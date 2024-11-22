delete(serialportfind);
portslist = serialportlist()
device = serialport(portslist(16), 115200)
flush(device);
device.UserData = struct("Data",[],"Count",1)
n = 0;
if exist('testdata.csv', 'file')==2
  delete('testdata.csv');
end
endstring = strcat(char(13), char(10));
maxlines = 10000000;
while (n < maxlines)
    string = readline(device)
    writelines(string, "testdata.csv", WriteMode="append");
    if(strcmp(string, char(13)))
        n = maxlines + 1;
        break;
    end
end