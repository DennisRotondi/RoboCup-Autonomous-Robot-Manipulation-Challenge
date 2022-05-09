function dydx = getGradientNet(net1, x)
%GETGRADIENT Summary of this function goes here
%   Detailed explanation goes here  
    % normalize x
%     nx = (x-net1.input.processSettings{1,1}.xmin).*net1.input.processSettings{1,1}.gain+net1.input.processSettings{1,1}.ymin;
%     h = tanh(net1.b{1}+net1.IW{1}*nx);             % h = [3xn] IW{1} = [3x1] x' = [1xn]
%     ny = net1.b{2}+net1.LW{2,1}*h;                    % y = [1xn]  LW{2,1} = [1x3]
%     % above ypredict is equivalent to predict(net1,x)
%     % derivative of nn at normalized scale    
%     dnydnx = sum(net1.LW{2,1}'.*net1.IW{1}.*(1-h.*h),1)';    % dyy = [1xn]  h'*h = [nxn]
%     % derivative of nn at real scale
%     dydx = dnydnx.*net1.input.processSettings{1,1}.gain./net1.output.processSettings{1,1}.gain;
    x = dlarray(x);
    [~, dydx] = dlfeval(@fun_and_deriv,x);

    function [y,dy] = fun_and_deriv(x)
        nx = (x-net1.input.processSettings{1,1}.xmin).*net1.input.processSettings{1,1}.gain+net1.input.processSettings{1,1}.ymin;
        h = tanh(net1.b{1}+net1.IW{1}*nx);             % h = [3xn] IW{1} = [3x1] x' = [1xn]
        ny = net1.b{2}+net1.LW{2,1}*h;                    % y = [1xn]  LW{2,1} = [1x3]
        % de-normalize y    
        y = (ny-net1.output.processSettings{1,1}.ymin)/net1.output.processSettings{1,1}.gain+net1.output.processSettings{1,1}.xmin;
        dy = dlgradient(y,x);
    end
end


