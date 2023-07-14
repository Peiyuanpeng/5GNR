function y=mseq(coef)  
m=length(coef);%确定寄存器数目  
N=2^m-1;%确定周期  
%mback=0;%用于存放反馈值  
y=zeros(1,N);%用于存放输出序列  
registers=[1,1,1,0,1,1,0];%确定寄存器初始值  
for i=1:N  
y(i)=registers(m);  
mback=mod(sum(coef.*registers),2);  
registers=[mback registers(1:end-1)];  
end  