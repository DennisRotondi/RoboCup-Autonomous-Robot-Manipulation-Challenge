function [prediction, bboxes, scores, labels] = classify(network,image)
    [prediction, bboxes, scores, labels] = network.predict(image);
    %the images is rescaled to 270x270 from 270x480, se we need to rescale
    %the image and the bboxes for the scaled dimension
    scaling=480/270;
    prediction=imresize(prediction,[270 480]);
    bboxes(:,[1,3])=bboxes(:,[1,3])*scaling;
end

