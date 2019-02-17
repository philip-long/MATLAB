function object_constants=getObjectParams(object_case)
% getObjectParams: Get object parameters, one of two configurations
% depending on object_case



if(object_case==1)
    
    object_constants.Table=[0.0 0.0 0.0
        0.6 0.0 0.0
        0.0 0.6 0.0
        0.6 0.6 0.0
        0.0 0.0 0.75
        0.6 0.0 0.75
        0.0 0.6 0.75
        0.6 0.6 0.75];
    object_constants.Book1 =[0.0 0.0 0.75
        0.1 0.0 0.75
        0.0 0.1 0.75
        0.1 0.1 0.75
        0.0 0.0 1.2
        0.1 0.0 1.2
        0.0 0.1 1.2
        0.1 0.1 1.2];
    
    object_constants.Book2=[0.2 0.5 0.75
        0.4 0.5 0.75
        0.2 0.6 0.75
        0.4 0.6 0.75
        0.2 0.5 1.2
        0.4 0.5 1.2
        0.2 0.6 1.2
        0.4 0.6 1.2];
    
    
    object_constants.Book3=[0.4 0.0 0.75
        0.6 0.0 0.75
        0.4 0.2 0.75
        0.6 0.2 0.75
        0.4 0.0 1.2
        0.6 0.0 1.2
        0.4 0.2 1.2
        0.6 0.2 1.2];
    
    
    
elseif(object_case==2)
    
    object_constants.Table=[0.0 0.0 0.0
        0.6 0.0 0.0
        0.0 0.6 0.0
        0.6 0.6 0.0
        0.0 0.0 0.75
        0.6 0.0 0.75
        0.0 0.6 0.75
        0.6 0.6 0.75];
    object_constants.Book1 =[0.0 0.0 0.75
        0.6 0.0 0.75
        0.0 0.1 0.75
        0.6 0.1 0.75
        0.0 0.0 1.2
        0.6 0.0 1.2
        0.0 0.1 1.2
        0.6 0.1 1.2];
    
    object_constants.Book2=[0.0 0.5 0.75
        0.6 0.5 0.75
        0.0 0.6 0.75
        0.6 0.6 0.75
        0.0 0.5 1.2
        0.6 0.5 1.2
        0.0 0.6 1.2
        0.6 0.6 1.2];
    object_constants.Book3=[0.0 0.0 0.75
        0.05 0.0 0.75
        0.0 0.6 0.75
        0.05 0.6 0.75
        0.0 0.0 0.85
        0.05 0.0 0.85
        0.0 0.6 0.85
        0.05 0.6 0.85];
end
end