function figureCallback(src,event)
% line = findobj(src,"Type","Line");
% if event.Character == "+"
%     line.LineWidth = line.LineWidth+1;
% elseif event.Character == "-"
%     line.LineWidth = max(line.LineWidth-1,0.5);
% end
% end

data = readline(src);
  % Hacer algo con los datos recibidos
  %disp(data);
  % Enviar una respuesta al ESP32
 
  try 
     src.UserData.Data(end+1) =  str2double(data);
     src.UserData.Count = src.UserData.Count + 1;
  catch
       disp(data);
  end

  plot(src.UserData.Data(2:end));