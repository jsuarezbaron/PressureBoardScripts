clear all
instrreset;
placa = serial('COM8','BaudRate',9600);  %Colocar el COM que aparece en Device Manager
fopen(placa);

%%%
figure(1)
hold on
tic
for i = 1:100
    str = query(placa, 'P');
    strP = sprintf('x%d',str );
    if (str(1) > 48 && str(1) < 57)
        presiones = str2num(str)
        tiempos = toc ;
        plot(tiempos, presiones(:,1),'.');      
    end
    print(strP)
    pause(1)
    
end

%%%
fclose(placa);
