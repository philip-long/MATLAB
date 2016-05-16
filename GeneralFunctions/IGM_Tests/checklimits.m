function Injointlimits=checklimits(Q)
% This function checks if configuration or set of configurations fall
% within the joint limits
% Inputs: Q an n*1 array
%         Q a cell contains x n*1 arrays

% Outputs a 1 is in joint limits a zero is outside
%
% To add: support if Q is an array of solutions rather than a set
% To add: Check all values in a trajectory 


global qlower qupper Joints

switch class(Q)
    
    case 'cell' % Input is a cell of solutions
        
        for i=1:size(Q,2)
            qc=Q{i};
            
            if size(qc,2)==6 % Transposing if data is wrong way around
                qc=qc';
            end
            
            if all([qc<qupper';qc>qlower'])
                Injointlimits(i)=1;
            else
                Injointlimits(i)=0;
            end
        end
        
        
        
        
    case 'double' % its an array
        
        if numel(Q)==6 % a single set
            
            qc=Q;
            
            if size(qc,2)==6 % Transposing if data is wrong way around
                qc=qc';
            end
            
            if all([qc<qupper';qc>qlower'])
                disp 'Solution Found'
                Injointlimits=1;
            else
                disp 'qc outside joint limits-restart'
                Injointlimits=0;
            end
            
        end
       
    otherwise
        disp 'Unrecognized input'
end