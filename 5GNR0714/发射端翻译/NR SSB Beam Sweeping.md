---
title: NR SSB Beam Sweeping translation
cover: 'https://img0.baidu.com/it/u=789232622,2333823016&fm=253&fmt=auto&app=138&f=JPEG?w=889&h=500'
description: 发送端
tags:
  - 学习
categories: 
  - 5G
  - MATLAB
---

## NR SSB波束扫频

本示例展示了如何在5G NR系统的发射端（gNB）和接收端（UE）采用波束扫描。

本示例使用同步信号块（SSB）说明了初始接入期间使用的一些波束管理程序。

为了完成波束扫描，本示例使用了相控阵系统工具箱™中的几个组件。

---

### 介绍

毫米波频率的支持需要定向链路，这导致了用于NR初始接入的波束管理程序的规范。

波束管理是一组第一层（物理）和第二层（介质访问控制）程序，用于获取和维护一组波束对链路（gNB使用的波束与UE使用的波束配对）。

波束管理程序适用于下行链路和上行链路的传输和接收[ 1 ]、[ 2 ]。**这些程序包括：**

* 波束扫频
* 波束测量
* 波束确定
* 波束检测
* 波束恢复

---

本示例侧重于在**用户设备（UE）**和接入网节点（gNB）之间建立连接时空闲用户的初始接入程序。

在物理层，使用同步信号块（SSB）作为下行链路方向（gNB到UE）的突发信号传输。

本示例突出了发射/接收点（TRP）波束扫描和UE波束扫描，以建立波束对链路。在多种波束管理程序中，TR 38.802将这种双端扫频定义为程序P-1[ 1 ]。

---

一旦连接，同一波束对链路可用于后续传输。如有必要，可使用CSI-RS（用于下行链路）和SRS（用于上行链路）进一步完善波束。如果波束失效，可重新建立波束对链路。有关波束对细化的示例，请参见使用CSI-RS的NR下行链路发送端**波束细化**。

[1]: https://ww2.mathworks.cn/help/5g/ug/nr-downlink-transmit-end-beam-refinement-using-csi-rs.html	"NR Downlink Transmit-End Beam Refinement Using CSI-RS"

---

* 该示例生成一个NR同步信号脉冲串，对脉冲串中的每个SSB进行波束成形，以扫过**方位角**和**仰角**方向，通过空间散射信道传输该波束成形信号，并通过**多个接收端波束**处理该接收信号。

* 该示例测量每个发射-接收波束对（双回路）的**参考信号接收功率（RSRP）**，并确定RSRP最大的波束对链路。
* 因此，该波束对链路是模拟空间场景下发射端和接收端的**最佳波束对**。该图显示了主要的处理步骤，波束管理步骤用彩色标出。

![流程图](https://picture-cloud-storage-pyp.oss-cn-beijing.aliyuncs.com/img/202307132155692.png)

```
rng(211);                           % 设置RNG状态以实现可重复性
```

---

### 模拟参数

为示例定义系统参数。修改这些参数，以了解它们对系统的影响。

```matlab
prm.NCellID = 1;                    % 小区 ID
prm.FreqRange = 'FR1';              % 频率范围: 'FR1' 或者 'FR2'
prm.CenterFreq = 3.5e9;             % 频率单位：赫兹
prm.SSBlockPattern = 'Case B';      % Case A/B/C/D/E
prm.SSBTransmitted = [ones(1,8) zeros(1,0)];   % 4/8 or 64 in length

prm.TxArraySize = [8 8];            % 传输数组大小，[行 列］
prm.TxAZlim = [-60 60];             % 发射方位角扫描限制
prm.TxELlim = [-90 0];              % 发射高程扫描限制

prm.RxArraySize = [2 2];            % 接收数组大小, [行列数]
prm.RxAZlim = [-180 180];           % 接收方位角扫描限制
prm.RxELlim = [0 90];               % 接收高程扫描限制

prm.ElevationSweep = false;         % 启用/禁用仰角扫描
prm.SNRdB = 30;                     % 信噪比, dB
prm.RSRPMode = 'SSSwDMRS';          % {'SSSwDMRS', 'SSSonly'}
```

示例中使用了这些参数：

* 单个BS和UE的单小区情况下的小区ID。
* 频率范围，以字符串形式指定 FR1 或 FR2 操作。
* 中心频率，单位Hz，取决于频率范围。
* 同步信号块模式，FR1 为情况 A/B/C，FR2 为情况 D/E。这也选择了子载波间隔。
* 模式中传输的 SSB，对于 FR1 为长度为 4 或 8 的二进制矢量，对于 FR2 为长度为 64 的二进制矢量。传输的 SSB 数设定发射端和接收端的波束数。
* 发射阵列尺寸，作为双元素行向量，分别指定发射阵列行和列的天线元素数。**当两个值都大于1时，使用均匀矩形阵列（URA）**。
* 发射方位角扫描限制（度），指定扫描的起始和终止方位角。
* 发射仰角扫描限值，单位为度，用于指定扫描的起始和终止仰角。
* 接收阵列尺寸，作为双元素行向量，分别指定接收阵列行和列的天线元素数。**当两个值都大于1时，使用均匀矩形阵列（URA）**。
* 以度为单位的接收方位角扫描限值，用于指定扫描的起始和终止方位角。
* 以度为单位的接收仰角扫描限值，用于指定扫描的起始和终止仰角。
* 启用或禁用发射端和接收端的仰角扫描。为 FR2 和/或 URA 启用仰角扫描。
* 信噪比（dB）。
* SSB 测量模式，指定仅使用辅助同步信号（'SSSonly'）或使用 PBCH DM-RS 以及辅助同步信号（'SSSwDMRS'）。

```matlab
prm = validateParams(prm);
```

validatteParams是一个本地函数，代码如下：

```matlab
function prm = validateParams(prm)
% 验证用户指定的参数并返回更新的参数。
%
% 仅对参数的一致性进行交叉检查。

    if strcmpi(prm.FreqRange,'FR1')
        if prm.CenterFreq > 7.125e9 || prm.CenterFreq < 410e6
            error(['Specified center frequency is outside the FR1 ', ...
                   'frequency range (410 MHz - 7.125 GHz).']);
        end
        if strcmpi(prm.SSBlockPattern,'Case D') ||  ...
           strcmpi(prm.SSBlockPattern,'Case E')
            error(['Invalid SSBlockPattern for selected FR1 frequency ' ...
                'range. SSBlockPattern must be one of ''Case A'' or ' ...
                '''Case B'' or ''Case C'' for FR1.']);
        end
        if ~((length(prm.SSBTransmitted)==4) || ...
             (length(prm.SSBTransmitted)==8))
            error(['SSBTransmitted must be a vector of length 4 or 8', ...
                   'for FR1 frequency range.']);
        end
        if (prm.CenterFreq <= 3e9) && (length(prm.SSBTransmitted)~=4)
            error(['SSBTransmitted must be a vector of length 4 for ' ...
                   'center frequency less than or equal to 3GHz.']);
        end
        if (prm.CenterFreq > 3e9) && (length(prm.SSBTransmitted)~=8)
            error(['SSBTransmitted must be a vector of length 8 for ', ...
                   'center frequency greater than 3GHz and less than ', ...
                   'or equal to 7.125GHz.']);
        end
    else % 'FR2'
        if prm.CenterFreq > 52.6e9 || prm.CenterFreq < 24.25e9
            error(['Specified center frequency is outside the FR2 ', ...
                   'frequency range (24.25 GHz - 52.6 GHz).']);
        end
        if ~(strcmpi(prm.SSBlockPattern,'Case D') || ...
                strcmpi(prm.SSBlockPattern,'Case E'))
            error(['Invalid SSBlockPattern for selected FR2 frequency ' ...
                'range. SSBlockPattern must be either ''Case D'' or ' ...
                '''Case E'' for FR2.']);
        end
        if length(prm.SSBTransmitted)~=64
            error(['SSBTransmitted must be a vector of length 64 for ', ...
                   'FR2 frequency range.']);
        end
    end

    prm.NumTx = prod(prm.TxArraySize);
    prm.NumRx = prod(prm.RxArraySize);
    if prm.NumTx==1 || prm.NumRx==1
        error(['Number of transmit or receive antenna elements must be', ...
               ' greater than 1.']);
    end
    prm.IsTxURA = (prm.TxArraySize(1)>1) && (prm.TxArraySize(2)>1);
    prm.IsRxURA = (prm.RxArraySize(1)>1) && (prm.RxArraySize(2)>1);

    if ~( strcmpi(prm.RSRPMode,'SSSonly') || ...
          strcmpi(prm.RSRPMode,'SSSwDMRS') )
        error(['Invalid RSRP measuring mode. Specify either ', ...
               '''SSSonly'' or ''SSSwDMRS'' as the mode.']);
    end

    % Select SCS based on SSBlockPattern
    switch lower(prm.SSBlockPattern)
        case 'case a'
            scs = 15;
            cbw = 10;
            scsCommon = 15;
        case {'case b', 'case c'}
            scs = 30;
            cbw = 25;
            scsCommon = 30;
        case 'case d'
            scs = 120;
            cbw = 100;
            scsCommon = 120;
        case 'case e'
            scs = 240;
            cbw = 200;
            scsCommon = 120;
    end
    prm.SCS = scs;
    prm.ChannelBandwidth = cbw;
    prm.SubcarrierSpacingCommon = scsCommon;

end
```

**代码理解：**

* 前半部分用于判断属于FR1频段还是FR2频段，如果在属于某一频段范围的前提下，其某项指标不满足相应的条件，则会报错。

* prm.NumTx 用于计算总天线数，必须要大于1，否则会报错。
* 文档中指示当两个值都大于1时，使用均匀矩形阵列（URA）。
* 下面进行SSB测量模式选择。
* 最后进行SSB锁定模式，通过case选择后，最后对参数进行赋值。

---

### 同步信号突发配置

使用指定的系统参数设置同步信号脉冲串参数。初始接入时，将SSB周期设为20ms。

```matlab
txBurst = nrWavegenSSBurstConfig;
txBurst.BlockPattern = prm.SSBlockPattern;
txBurst.TransmittedBlocks = prm.SSBTransmitted;
txBurst.Period = 20;
txBurst.SubcarrierSpacingCommon = prm.SubcarrierSpacingCommon;

% 配置 nrDLCarrierConfig 对象以使用同步信号
% 该对象用于设置突发参数和禁用其他通道。nrWaveformGenerator 将使用该对象生成 SS burst 波形。

cfgDL = configureWaveformGenerator(prm,txBurst);
```

首先查询 *nrWavegenSSBurstConfig* 是什么：

*nrWavegenSSBurstConfig* ：用于生成5G波形的SS突发配置参数

**说明：**

* nrWavegenSSBurstConfig 对象设置同步信号（SS）突发配置参数。在配置 5G 下行链路波形生成时，使用该对象设置 nrDLCarrierConfig 对象的 SSBurst 属性。

* 该对象定义 SS burst 的子载波间隔 (SCS)、时域和频域分配、功率和有效载荷。

* 默认的 nrWavegenSSBurstConfig 对象将 SS burst 配置为四个活动 SS 块，周期为 20 ms，与初始小区选择相对应。默认配置还指定SS突发携带主信息块（MIB），并将SS突发置于子载波间隔为15 kHz的载波中心（块模式情况A）。要更新SS突发的频率位置，请将NCRBSSB和KSSB对象属性设置为非空值。

![参数1](https://picture-cloud-storage-pyp.oss-cn-beijing.aliyuncs.com/img/202307142017047.png)

![参数2](https://picture-cloud-storage-pyp.oss-cn-beijing.aliyuncs.com/img/202307142017275.png)

---

下面是另一个本地函数configureWaveformGenerator：

```matlab
function cfgDL = configureWaveformGenerator(prm,txBurst)
% 配置nrWaveformGenerator使用的nrDLCarrierConfig对象。
% 产生SS脉冲串波形。

    cfgDL = nrDLCarrierConfig;
    cfgDL.SCSCarriers{1}.SubcarrierSpacing = prm.SCS;
    if (prm.SCS==240)
        cfgDL.SCSCarriers = [cfgDL.SCSCarriers cfgDL.SCSCarriers];
        cfgDL.SCSCarriers{2}.SubcarrierSpacing = prm.SubcarrierSpacingCommon;
        cfgDL.BandwidthParts{1}.SubcarrierSpacing = prm.SubcarrierSpacingCommon;
    else
        cfgDL.BandwidthParts{1}.SubcarrierSpacing = prm.SCS;
    end
    cfgDL.PDSCH{1}.Enable = false;
    cfgDL.PDCCH{1}.Enable = false;
    cfgDL.ChannelBandwidth = prm.ChannelBandwidth;
    cfgDL.FrequencyRange = prm.FreqRange;
    cfgDL.NCellID = prm.NCellID;
    cfgDL.NumSubframes = 5;
    cfgDL.WindowingPercent = 0;
    cfgDL.SSBurst = txBurst;

end
```

![属性列表](https://picture-cloud-storage-pyp.oss-cn-beijing.aliyuncs.com/img/202307142021729.png)

---

### 脉冲发生器

调用 nrWaveformGenerator 函数创建 SS burst 波形[ 3 ]。生成的波形尚未波束成形。

```matlab
burstWaveform = nrWaveformGenerator(cfgDL);

% 显示SS脉冲串波形的频谱图
figure;
ofdmInfo = nrOFDMInfo(cfgDL.SCSCarriers{1}.NSizeGrid,prm.SCS);
nfft = ofdmInfo.Nfft;
spectrogram(burstWaveform,ones(nfft,1),0,nfft,'centered',ofdmInfo.SampleRate,'yaxis','MinThreshold',-130);
title('Spectrogram of SS burst waveform')
```

最后可以跑出来这样一个代码，应该是SSB的样子了

![image-20230714213556262](https://picture-cloud-storage-pyp.oss-cn-beijing.aliyuncs.com/img/202307142135297.png)

---

### 通道配置

* 配置空间散射 MIMO 信道。该信道模型对输入应用**自由空间路径损耗**和可选的**其他大气衰减**。

* 在直角坐标系中以[x,y,z]坐标指定**BS**和**UE**的位置。

* 根据指定的阵列尺寸，采用**均匀线性阵列（ULA）**或**均匀矩形阵列（URA）**。

* 阵列采用**各向同性**天线元件。

```matlab
c = physconst('LightSpeed');   % 传播速度
lambda = c/prm.CenterFreq;     % 波长

prm.posTx = [0;0;0];           % 发射阵列位置，[x;y;z]，米
prm.posRx = [100;50;0];        % 接收阵列位置，[x;y;z]，米

toRxRange = rangeangle(prm.posTx,prm.posRx);
% rangeangle 函数用于确定信号从一个源点或一组源点到一个参考点的传播路径长度和路径方向。
spLoss = fspl(toRxRange,lambda);   % 自由空间路径损耗

% 发射阵列
if prm.IsTxURA
    % 均匀矩形阵列
    arrayTx = phased.URA(prm.TxArraySize,0.5*lambda, ...
        'Element',phased.IsotropicAntennaElement('BackBaffled',true));
else
    % 均匀线性阵列
    arrayTx = phased.ULA(prm.NumTx, ...
        'ElementSpacing',0.5*lambda, ...
        'Element',phased.IsotropicAntennaElement('BackBaffled',true));
end

% 接收阵列
if prm.IsRxURA
    % 均匀矩形阵列
    arrayRx = phased.URA(prm.RxArraySize,0.5*lambda, ...
        'Element',phased.IsotropicAntennaElement);
else
    % 均匀线性阵列
    arrayRx = phased.ULA(prm.NumRx, ...
        'ElementSpacing',0.5*lambda, ...
        'Element',phased.IsotropicAntennaElement);
end

% 散射器位置
prm.FixedScatMode = true;
if prm.FixedScatMode
    % 固定单个散射体位置
    prm.ScatPos = [50; 80; 0];
else
    % 在随机位置产生散射体
    Nscat = 10;        % 散射体数量
    azRange = -180:180;
    elRange = -90:90;
    randAzOrder = randperm(length(azRange));
    randElOrder = randperm(length(elRange));
    azAngInSph = azRange(randAzOrder(1:Nscat));
    elAngInSph = elRange(randElOrder(1:Nscat));
    r = 20;            % 半径
    [x,y,z] = sph2cart(deg2rad(azAngInSph),deg2rad(elAngInSph),r);
    prm.ScatPos = [x;y;z] + (prm.posTx + prm.posRx)/2;
end

% 配置通道
channel = phased.ScatteringMIMOChannel;
channel.PropagationSpeed = c;
channel.CarrierFrequency = prm.CenterFreq;
channel.SampleRate = ofdmInfo.SampleRate;
channel.SimulateDirectPath = false;
channel.ChannelResponseOutputPort = true;
channel.Polarization = 'None';
channel.TransmitArray = arrayTx;
channel.TransmitArrayPosition = prm.posTx;
channel.ReceiveArray = arrayRx;
channel.ReceiveArrayPosition = prm.posRx;
channel.ScattererSpecificationSource = 'Property';
channel.ScattererPosition = prm.ScatPos;
channel.ScattererCoefficient = ones(1,size(prm.ScatPos,2));

% 获取最大通道延迟
[~,~,tau] = channel(complex(randn(ofdmInfo.SampleRate*1e-3,prm.NumTx), ...
    randn(ofdmInfo.SampleRate*1e-3,prm.NumTx)));
maxChDelay = ceil(max(tau)*ofdmInfo.SampleRate);
```

---

### 发射端波束扫描

* 为实现TRP波束扫描，使用模拟波束成形对生成的突发中的**每个SS块进行波束成形**。

* 根据突发中SS块的数量和指定的扫描范围，确定不同波束的**方位角**和**仰角**方向。

* 然后将脉冲串中的各个块波束**成形到每个方向**。

```matlab
% 发射端和接收端的波束数
numBeams = sum(txBurst.TransmittedBlocks);

% 等间距方位角和仰角发射光束
azBW = beamwidth(arrayTx,prm.CenterFreq,'Cut','Azimuth');
elBW = beamwidth(arrayTx,prm.CenterFreq,'Cut','Elevation');
txBeamAng = hGetBeamSweepAngles(numBeams,prm.TxAZlim,prm.TxELlim, ...
    azBW,elBW,prm.ElevationSweep);

% 用于评估发射侧转向权重
SteerVecTx = phased.SteeringVector('SensorArray',arrayTx, ...
    'PropagationSpeed',c);

% 获取每个SSB占用的OFDM符号集
numBlocks = length(txBurst.TransmittedBlocks);
burstStartSymbols = ssBurstStartSymbols(txBurst.BlockPattern,numBlocks);
burstStartSymbols = burstStartSymbols(txBurst.TransmittedBlocks==1);
burstOccupiedSymbols = burstStartSymbols.' + (1:4);

% 为每个SSB的每个OFDM符号应用转向
gridSymLengths = repmat(ofdmInfo.SymbolLengths,1,cfgDL.NumSubframes);
% 在numTx上重复突发，为转向做准备
strTxWaveform = repmat(burstWaveform,1,prm.NumTx)./sqrt(prm.NumTx);
for ssb = 1:numBeams

    % 从脉冲串中提取SSB波形
    blockSymbols = burstOccupiedSymbols(ssb,:);
    startSSBInd = sum(gridSymLengths(1:blockSymbols(1)-1))+1;
    endSSBInd = sum(gridSymLengths(1:blockSymbols(4)));
    ssbWaveform = strTxWaveform(startSSBInd:endSSBInd,1);

    % 生成转向权重
    wT = SteerVecTx(prm.CenterFreq,txBeamAng(:,ssb));

    % 将每个发射单元的权重应用于 SSB
    strTxWaveform(startSSBInd:endSSBInd,:) = ssbWaveform.*(wT');

end
```

---

下面是接收端的，明天再看

### 接收端波束扫描和测量

### 波束确定





