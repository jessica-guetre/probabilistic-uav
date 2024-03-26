function roi = getTarget(probabilityGrid, targetFactor)
% Determines a target region of interest (ROI) within a probability grid by selecting a cell based on weighted probabilities.
%
% Inputs:
% - probabilityGrid (matrix): Grid of probabilities representing the likelihood of each cell being the target location.
% - targetFactor (scalar): Exponent used to adjust the weighting of probabilities in the grid.
%
% Output:
% - roi (vector): Defines the region of interest for the target as [startRow, endRow, startCol, endCol, minHeight, maxHeight].
%                 The target region is selected based on the adjusted probabilities, ensuring a random but biased selection towards higher probabilities.
%

    probabilityGrid = probabilityGrid.^targetFactor; % increase weighting of probability
    probabilities = probabilityGrid / sum(probabilityGrid, 'all'); % normalize to sum to 1
    cumulativeProbabilities = cumsum(probabilities(:)); % converto to cumulative sum array
    randValue = rand(); % generate a random value and find the first index where the cumulative
    targetCellIndex = find(cumulativeProbabilities >= randValue, 1, 'first'); % select index for first occurrence
    [row, col] = ind2sub(size(probabilityGrid), targetCellIndex); % convert the 1D index back to 2D
    if row == size(probabilityGrid, 1)
        row = row - 1;
    end
    if col == size(probabilityGrid, 2)
        col = col - 1;
    end
    roi = [row, row + 1, col, col + 1, 0, 3]; % define ROI
end