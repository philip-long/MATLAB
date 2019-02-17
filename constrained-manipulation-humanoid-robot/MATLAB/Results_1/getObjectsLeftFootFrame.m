function [ objects ] = getObjectsLeftFootFrame( Lf_T_opt,Table,Book1,Book2,Book3 )
%getObjectsLeftFootFrame Transform the objects from object frame to left
%foot frame

lf_Table= Table; 
lf_Book1=Book1;
lf_Book2=Book2;
lf_Book3=Book3;

for i=1:length(Table)
    temp=Lf_T_opt*[Table(i,:),1]';
    lf_Table(i,:)=temp(1:3);
    temp=Lf_T_opt*[Book1(i,:),1]';
    lf_Book1(i,:)=temp(1:3);
    temp=Lf_T_opt*[Book2(i,:),1]';
    lf_Book2(i,:)=temp(1:3);
    temp=Lf_T_opt*[Book3(i,:),1]';
    lf_Book3(i,:)=temp(1:3);
end
objects{1}=Polyhedron(lf_Table);
objects{2}=Polyhedron(lf_Book1);
objects{3}=Polyhedron(lf_Book2);
objects{4}=Polyhedron(lf_Book3);


end

