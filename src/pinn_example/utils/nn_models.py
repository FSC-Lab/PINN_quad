"""
define network for training
"""
import torch
import torch.nn as nn

class MetricNet(nn.Module):
    def __init__(self):
        super(MetricNet, self).__init__()
        self.hidden_layer1 = nn.Linear(1, 5)
        self.hidden_layer2 = nn.Linear(5, 5)
        self.hidden_layer3 = nn.Linear(5, 5)
        self.hidden_layer4 = nn.Linear(5, 5)
        self.hidden_layer5 = nn.Linear(5, 5)
        self.output_layer = nn.Linear(5, 1)

    def forward(self, x):
        layer1_out = torch.sigmoid(self.hidden_layer1(x))
        layer2_out = torch.sigmoid(self.hidden_layer2(layer1_out))
        layer3_out = torch.sigmoid(self.hidden_layer3(layer2_out))
        layer4_out = torch.sigmoid(self.hidden_layer4(layer3_out))
        layer5_out = torch.sigmoid(self.hidden_layer5(layer4_out))
        output = self.output_layer(layer5_out) ## For regression, no activation is used in output layer
        return output
    
class MetricNet2(nn.Module):
    def __init__(self, input_dim, hidden_layers=[20, 20, 20],
                 output_dim=1, dropout_prob=0.2):
        """
        Parameters:
            - input_dim: Number of input features
            - hidden_layers: List specifying the number of neurons in each hidden layer
            - output_dim: Number of output neurons (default=1 for regression)
            - dropout_prob: Dropout probability for regularization (default=0.2)
        """
        super(MetricNet2, self).__init__()
        
        self.layers = nn.ModuleList()  # Store layers dynamically
        prev_dim = input_dim
        
        # Create hidden layers
        for layer_size in hidden_layers:
            self.layers.append(nn.Linear(prev_dim, layer_size))
            prev_dim = layer_size  # Update previous layer size

        self.output_layer = nn.Linear(prev_dim, output_dim)
        self.dropout = nn.Dropout(p=dropout_prob)

        # Initialize weights
        self.init_weights()

    def init_weights(self):
        """Applies Kaiming He initialization for stability with ReLU."""
        for layer in self.layers:
            if isinstance(layer, nn.Linear):
                nn.init.kaiming_uniform_(layer.weight, nonlinearity='relu')
                nn.init.zeros_(layer.bias)
        nn.init.kaiming_uniform_(self.output_layer.weight, nonlinearity='linear')
        nn.init.zeros_(self.output_layer.bias)

    def forward(self, x):
        for layer in self.layers:
            x = torch.relu(layer(x))
            x = self.dropout(x)  # Apply dropout

        output = self.output_layer(x)  # No activation for regression
        return output