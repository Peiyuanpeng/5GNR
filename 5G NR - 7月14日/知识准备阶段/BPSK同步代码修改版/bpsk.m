m_bin= zeros(3,127);  
m_bin(1,:) = mseq([0 0 0 0 0 1 1]); % 生成127位m序列
m_bin(2,:) = mseq([0 0 0 1 0 0 1]); % 生成127位m序列  
m_bin(3,:) = mseq([0 0 0 1 1 1 1]); % 生成127位m序列  
m_data = 2 * [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 m_bin(1,:) m_bin(2,:) m_bin(3,:)] -1; % 转换为±1，把三个m序列拼接在一起发送  
m_data = [m_data m_data]; % 重复两次  
N=length(m_data);   
Tc=1e-6;%码字周期   
f=2e6;%载波频率  
Fs=20;%载波周期点数  
N_c = Tc * f * N;     % 基带数据对应载波周期数  
% 先采用BPSK对发送的m序列进行调制，并在调制信号上叠加加性高斯白噪声，信噪比设置为为5dB。然后在接收端使用BPSK对信号进行解调，如图所示，可以看到调制后的数据及解调后的数据由于干扰比较大，其实很难在解调后的基带数据上进行逐次比较法确定同步序列。
% BPSK调制；  
tx_data= zeros(N_c * Fs,1);  
for i = 1: N  
    bit_idx = 1:1: Tc * f * Fs;  
    tx_data((i-1)*length(bit_idx) + bit_idx) = m_data(i) * cos(2 * pi / Fs * bit_idx);  
end  
channel_data = awgn(tx_data,-5, 'measured'); % 增加噪声,5dB  
% BPSK解调过程  
rx_data = zeros(N,1); 
for i=1:N  
    bit_idx = 1:1:Tc * f * Fs ;  
    %channel_data数据与cos(2f*pi*t)相乘  
    rx_data_with_4pift = channel_data((i-1)*length(bit_idx) + bit_idx) .* cos(2 * pi / Fs * bit_idx)';  
    %积分过滤出基带分量  
    rx_data(i) = sum(rx_data_with_4pift);
    if(rx_data(i)) >= 0
        rx_data(i) = 1;
    else rx_data(i) = 0;
    end
end  

coef= zeros(3,127);  
for i=1:127  
     coef(1,i) = min(min(corrcoef(rx_data(i:i+126), m_bin(1,:) )));
     coef(2,i) = min(min(corrcoef(rx_data(i:i+126), m_bin(2,:) )));  
     coef(3,i) = min(min(corrcoef(rx_data(i:i+126), m_bin(3,:) ))); 
end 

x = 1:127;
figure
subplot(3,1,1);
plot(x,coef(1,:));
ylim([-0.3 1])

subplot(3,1,2)
plot(x,coef(2,:));
ylim([-0.3 1]) 

subplot(3,1,3)
plot(x,coef(3,:));
ylim([-0.3 1]) 
