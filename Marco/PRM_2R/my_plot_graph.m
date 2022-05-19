function [outputArg1,outputArg2] = my_plot_graph(G)
%MY_PLOT_GRAPH Summary of this function goes here
%   Detailed explanation goes here
    figure;
    hold on;
    for i=1:G.numnodes
        plot(G.Nodes.coords{i}(1),G.Nodes.coords{i}(2),'o','Color',[0.2,0.2,1]);
    end
    for i=1:G.numedges
        idxs = G.Edges(i,:);
        idx1 = idxs.('EndNodes')(1);
        idx2 = idxs.('EndNodes')(2);
        p1 = G.Nodes.coords{idx1};
        p2 = G.Nodes.coords{idx2};
        plot([p1(1),p2(1)],[p1(2),p2(2)],'Color',[0.2,0.2,1]);
    end
    xlim([-pi,+pi]);
    ylim([-pi,+pi]);
end

