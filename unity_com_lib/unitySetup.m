function client = unitySetup(port, time_out)
     if ~exist('time_out', 'var') 
         time_out = 30;
     end
     client = tcpip('127.0.0.1',port,'NetworkRole','Client');
     set(client,'Timeout',time_out);
end