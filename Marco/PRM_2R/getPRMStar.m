function G = getPRMStar(robot, n_points)
%GETPRMSTAR Summary of this function goes here
%   Detailed explanation goes here
    G = graph;
    yprm = pi;
    minDet = 0.01;
    while true
        Nnew = randomConfiguration(robot);
        newNode = table({Nnew},'VariableNames',{'coords'});
        J = geometricJacobian(robot,Nnew,'endeffector');
        J = J(4:5,:);
        detJ = det(J);
        if abs(detJ)>minDet
            G = addnode(G,newNode);
            break
        end
    end
    itr = 2; %for the radius it starts at two
    while itr<=n_points
        Nnew = randomConfiguration(robot);
        isNew = true;
        radius = yprm*(log(itr)/itr)^(1/length(Nnew));
        for i=1:G.numnodes
            distNodes = norm(Nnew-G.Nodes.coords{i});
            if distNodes<radius && checkPath(robot,Nnew,G.Nodes.coords{i},minDet)
                if isNew
                    newNode = table({Nnew},'VariableNames',{'coords'});
                    G = addnode(G,newNode);
                    isNew = false;
                    itr = itr+1;
                end
                G = addedge(G,i,G.numnodes,distNodes);
            end
        end
    end
end

function isValid = checkPath(robot,qs1,qs2,minDet)
    dq = 5e-2;
    deltaq = norm(qs1-qs2);
    n_iter = deltaq/dq;
    isValid = true;
    for i = 0:1/n_iter:1
        checkq = (1-i)*qs1+i*qs2;
        J = geometricJacobian(robot,checkq,'endeffector');
        J = J(4:5,:);
        detJ = det(J);
        if abs(detJ)<minDet
            isValid = false;
        end
    end
end
