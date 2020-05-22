function display_decomposed_map(decomposed_map)
% display_decomposed_map Displays the cells of a decomposed map as different
% grayscale shades
%   Detailed explanation goes here

imshow(decomposed_map, [min(decomposed_map(:)),max(decomposed_map(:))])

end

