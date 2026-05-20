        clear all
        close all

        % Create positions for driving a dc-motor back and forth
        %Parametrar
        StartPos = 0;
        MaxAcc = 10000; % Acceleration
        DeAcc = -10000; % Deacceleration
        MaxSpeed = 3000; % Max Speed
        
        h = 0.01; % SampleTime
        Gear_Reduction = 75/19;
        Encoder = 128;
        Encoder_Counter = 4;
        Nbr_Puls_Per_Turn = Encoder*Encoder_Counter*Gear_Reduction
        EndPos = Nbr_Puls_Per_Turn;
        % Init
        BreakPos=0; 
        Position = StartPos;
        Speed = 0;
        OldSpeed = 0;
        n = 0;

        % Program
        if EndPos > StartPos
            while EndPos > Position && Speed >= 0
           %BreakTime = Speed/MinAcc; % Time for brake [Time = Speed/acc]
           %BreakPos = Breake Time * Speed/2;
            % Calculation
                if Speed < MaxSpeed & (Position  < EndPos + BreakPos)
                    Speed = OldSpeed + MaxAcc * h;
                    BreakPos = Speed*Speed/(2*DeAcc);
                elseif Position  >  EndPos + BreakPos
                    Speed = OldSpeed + DeAcc * h;
                else
                    Speed = MaxSpeed;
                end
                Position = Position + Speed * h;

                % save variable for plotting
                n = n + 1; 
                Pos(n) = Position;
                Sp(n) = Speed;
                Acc(n) = (Speed-OldSpeed)/h;

                % Uppdate OldSpeed
                OldSpeed = Speed;
            end
        end
        %else
        EndPos = 0;
        Speed =0;
            while Position > EndPos && Speed <= 0 
           %BreakTime = Speed/MinAcc; % Time for brake [Time = Speed/acc]
           %BreakPos = Breake Time * Speed/2;
            % Calculation
                if Speed > -MaxSpeed & Position > (EndPos + BreakPos)
                    Speed = (OldSpeed - MaxAcc * h);
                    BreakPos = -Speed*Speed/(2*DeAcc);
                elseif Position < (EndPos + BreakPos)
                    Speed = (OldSpeed - DeAcc * h);
                else
                    Speed = -MaxSpeed;
                end
                Position = Position + Speed * h;

                % save variable for plotting
                n = n + 1; 
                Pos(n) = Position;
                Sp(n) = Speed;
                Acc(n) = (Speed-OldSpeed)/h;

                % Uppdate OldSpeed
                OldSpeed = Speed;
            end            
    %     end

        %Plot
        subplot(3,1,1)
        plot((1:length(Pos))*h,Pos)
 %       axis([0 2.5 -500 5000]);
        legend('Position')
        subplot(3,1,2)
        plot((1:length(Sp))*h,Sp)
  %      axis([0 2.5 0 2300]);
        legend('Speed')
        subplot(3,1,3)
        plot((1:length(Acc))*h,Acc)
   %     axis([0 2.5 -4500 5000]);
        legend('Acceleration')
        xlabel('time [s]');
        
        

fileID = fopen('positions.txt','w');
fprintf(fileID,'%d\n',round(Pos));
fclose(fileID);
        