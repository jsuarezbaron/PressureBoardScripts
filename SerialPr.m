%%%
%
placa = serial('COM5','BaudRate',9600);
fopen(placa);

%%
%

figure(1)
hold on
tic
for i = 1:100
    
    str = query(placa, 'P');
    if (str == '')
    else
        presiones = str2num(str)
        tiempos = toc ;
        plot(tiempos, presiones(:,1),'.');
    end
    pause(1)
    
end
