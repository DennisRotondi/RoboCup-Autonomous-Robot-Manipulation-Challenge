function [NNLayer] = CreateLayers()
%CREATELAYERS Summary of this function goes here
%   Detailed explanation goes here
    inputLayer = imageInputLayer([270 270 3], "Name", "Input", "Normalization","none"); 
                filter_size = [3 3]; 
                %%
    middleLayer = [
                    convolution2dLayer(filter_size, 16, "Padding",1, "Name", "conv_1",...
                    "WeightsInitializer","narrow-normal")
                    batchNormalizationLayer("Name", "BN1")
                    reluLayer("Name", "relu_1")
                    maxPooling2dLayer(2, "Stride", 5,  "Name", "maxpool1")
                    
                    convolution2dLayer(filter_size, 16, "Padding",1, "Name", "conv_2",...
                    "WeightsInitializer","narrow-normal")
                    batchNormalizationLayer("Name", "BN2")
                    reluLayer("Name", "relu_2")
                    maxPooling2dLayer(2, "Stride", 3,  "Name", "maxpool2")
                    
                    convolution2dLayer(filter_size, 16, "Padding",1, "Name", "conv_3",...
                    "WeightsInitializer","narrow-normal")
                    batchNormalizationLayer("Name", "BN3")
                    reluLayer("Name", "relu_3")
                    
                    convolution2dLayer(filter_size, 16, "Padding",1, "Name", "conv_4",...
                    "WeightsInitializer","narrow-normal")
                    batchNormalizationLayer("Name", "BN4")
                    reluLayer("Name", "relu_4")
                    
                    ]; 

    NNLayer = [
       inputLayer; 
       middleLayer
    ]; 
end

