function G = getPRMStar(robot, n_points)
%GETPRMSTAR Summary of this function goes here
%   Detailed explanation goes here
    G = graph;
    yprm = pi;
    minDet = 0.001;
    while true
        Nnew = randomConfiguration(robot);
        newNode = table({Nnew},'VariableNames',{'coords'});
        J = geometricJacobian(robot,Nnew,'gripper');
        J = J(1:6,1:6);
        detJ = det(J);
        if abs(detJ)>minDet
            G = addnode(G,newNode);
            break
        end
    end
    disp('Starting point chosen');
    itr = 1;
    while itr<n_points
        Nnew = randomConfiguration(robot);
        isNew = true;
        radius = yprm*(log(itr+1)/(itr+1))^(1/length(Nnew));
        for i=1:G.numnodes
            distNodes = norm(Nnew-G.Nodes.coords{i});
            if distNodes<radius && checkPath(robot,Nnew,G.Nodes.coords{i},minDet)
                if isNew
                    newNode = table({Nnew},'VariableNames',{'coords'});
                    G = addnode(G,newNode);
                    isNew = false;
                    itr = itr+1;
                    clc;
                    disp(itr);
                end
                G = addedge(G,i,G.numnodes,distNodes);
            end
        end
    end
end

function isValid = checkPath(robot,qs1,qs2,minDet)
    dq = 1e-1;
    deltaq = norm(qs1-qs2);
    n_iter = deltaq/dq;
    isValid = true;
    for i = 0:1/n_iter:1
        checkq = (1-i)*qs1+i*qs2;
        J = geometricJacobian(robot,checkq,'gripper');
        J = J(1:6,1:6);
        detJ = det(J);
        if abs(detJ)<minDet
            isValid = false;
%             disp('Path not valid')
%             disp(detJ)
%             disp(i)
            break;
        end
    end
end
