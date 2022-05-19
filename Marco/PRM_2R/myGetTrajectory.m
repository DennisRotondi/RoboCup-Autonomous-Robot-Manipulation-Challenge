function traj = myGetTrajectory(G, start, target)
%MYGETTRAJECTORY Summary of this function goes here
%   Detailed explanation goes here
    [startNode,distStart] = getCloser(G,start);
    [endNode,distEnd] = getCloser(G,target);

    pathIdxs = shortestpath(G,startNode,endNode);

    totalLength = getTotalLength(G,pathIdxs,start,target);

    tot_points = 90;
    n_points = ceil(tot_points*distStart/totalLength);
    s = 0:1/n_points:1;
    traj = (1-s)'.*start+s'.*G.Nodes.coords{startNode};
    for i=1:length(pathIdxs)-1
        edge = findedge(G,pathIdxs(i),pathIdxs(i+1));
        dist = G.Edges.Weight(edge);
        n_points = ceil(tot_points*dist/totalLength);
        s = 0:1/n_points:1;
        traj = [traj ; ((1-s)'.*G.Nodes.coords{pathIdxs(i)}+s'.*G.Nodes.coords{pathIdxs(i+1)})];
    end
    n_points = ceil(tot_points*distEnd/totalLength);
    s = 0:1/n_points:1;
    traj= [traj ; ((1-s)'.*G.Nodes.coords{endNode}+s'.*target)];
    traj = traj(round(1:length(traj)/tot_points:length(traj)),:);

    window = 5;
    traj = smoothdata([ones(window,2).*traj(1,:); traj; ones(window,2).*traj(end,:)],1,'gaussian',int8(1.5*window));
end

function [idxCloser,minDist] = getCloser(G,point)
    minDist = inf;
    idxCloser = 1;
    for i=1:G.numnodes
        dist = norm(point-G.Nodes.coords{i});
        if dist < minDist
            idxCloser = i;
            minDist = dist;
        end
    end
end

function tot = getTotalLength(G,pathIdxs, startPoint, endPoint)
    tot = 0;
    tot = tot + norm(startPoint-G.Nodes.coords{pathIdxs(1)});
    tot = tot + norm(endPoint-G.Nodes.coords{pathIdxs(end)});
    for i=1:length(pathIdxs)-1
        edge = findedge(G,pathIdxs(i),pathIdxs(i+1));
        tot = tot + G.Edges.Weight(edge);
    end
end