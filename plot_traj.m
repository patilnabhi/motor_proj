function   data = plot_traj(mySerial)
  nsamples = fscanf(mySerial,'%d');
  data = zeros(nsamples,2);
  for i=1:nsamples
    data(i,:) = fscanf(mySerial,'%d %d');
    times(i) = (i-1)*0.005;
  end
  if nsamples > 1
    stairs(times,data(:,1:2));
    
  else
    fprintf('Only 1 sample received\n\n')
    disp(data);
  end
  % compute the control score, assuming that the first column of data
  % is the reference and the second is the sensor reading
  % score = norm(data(:,1)-data(:,2),1);
  score = mean(abs(data(:,1)-data(:,2)));
  fprintf('\nScore: %5.3f\n\n',score);
  title(sprintf('Average error: %5.2f deg',score));
  xlabel('time (s)'); 
  ylabel('angle (deg)') 
  legend('Reference','Trajectoy')
end
