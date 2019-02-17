function [ objects ] = getObjectsLeftFootFrame( Lf_T_opt,object_constants )
%getObjectsLeftFootFrame Transform the objects from object frame to left
%foot frame

lf_object_constants.Table= object_constants.Table; 
lf_object_constants.Book1=object_constants.Book1;
lf_object_constants.Book2=object_constants.Book2;
lf_object_constants.Book3=object_constants.Book3;

for i=1:length(object_constants.Table)
    temp=Lf_T_opt*[object_constants.Table(i,:),1]';
    lf_object_constants.Table(i,:)=temp(1:3);
    temp=Lf_T_opt*[object_constants.Book1(i,:),1]';
    lf_object_constants.Book1(i,:)=temp(1:3);
    temp=Lf_T_opt*[object_constants.Book2(i,:),1]';
    lf_object_constants.Book2(i,:)=temp(1:3);
    temp=Lf_T_opt*[object_constants.Book3(i,:),1]';
    lf_object_constants.Book3(i,:)=temp(1:3);
end
objects{1}=Polyhedron(lf_object_constants.Table);
objects{2}=Polyhedron(lf_object_constants.Book1);
objects{3}=Polyhedron(lf_object_constants.Book2);
objects{4}=Polyhedron(lf_object_constants.Book3);


end

