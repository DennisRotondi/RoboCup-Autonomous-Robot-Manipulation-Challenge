

classdef YOLOv2Net	
    
    properties 
        Name
        Anchor 
        DataStore
        Detector
        LGraph
    end 
    
    methods 
        function obj = YOLOv2Net(name)
            obj.Name = name; 
            disp("Created NN"); 
        end 
        
         function obj = set.DataStore(obj, ds)
            obj.DataStore = ds; 
            disp(obj)
         end 
         
         function obj = set.LGraph(obj, ds)
            obj.LGraph = ds; 
             
         end 
        
        function [img, bboxes, scores, labels] = predict(obj, image)
           img = imresize(image, [270,270]); % Resize (Encode) 
           [bboxes, scores, labels] = detect(obj.Detector, img); 

            % figure; 
            if(~isempty(bboxes))
                img = insertObjectAnnotation(img,'rectangle',bboxes,labels);
            end

            % imshow(img)
           
        end
    end 
end 