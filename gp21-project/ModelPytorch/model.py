import torch
import torch_geometric
import torch.nn as nn
import numpy as np
from torch_geometric.transforms import FaceToEdge

class ResConv(torch.nn.Module):
    def __init__(self, c_in, c_out, kernel):
        super(ResConv, self).__init__()
        self.shortcut = torch_geometric.nn.ChebConv(c_in, c_out, 1)
        self.conv1 = torch_geometric.nn.ChebConv(c_in, c_out, kernel)
        self.conv2 = torch_geometric.nn.ChebConv(c_out, c_out, kernel)

    def forward(self, x, edgeIndex):
        shortcut = self.shortcut(x, edgeIndex)
        x = self.conv1(x, edgeIndex)
        x = nn.functional.leaky_relu(x)
        x = self.conv2(x, edgeIndex)
        return x+shortcut


class Encoder(torch.nn.Module):
    def __init__(self, downscale):
        super(Encoder, self).__init__()
        self.downscale = downscale

        # Encoder
        self.conv1_enc = torch_geometric.nn.ChebConv(3, 16, 6)
        self.conv2_enc = torch_geometric.nn.ChebConv(16, 16, 6)
        self.conv3_enc = torch_geometric.nn.ChebConv(16, 16, 6)
        self.conv4_enc = torch_geometric.nn.ChebConv(16, 32, 6)
        self.fc_enc = nn.Linear(704, 8)


        """self.resconv1_enc = ResConv(3, 16, 6)
        self.resconv2_enc = ResConv(16, 16, 6)
        self.resconv3_enc = ResConv(16, 32, 6)
        self.resconv4_enc = ResConv(32, 32, 6)"""


    def forward(self, data):

        x, edge_index, pos = data.x, data.edge_index, data.pos

        x = self.conv1_enc(x, edge_index)
        x = nn.functional.leaky_relu(x)
        
        x = self.downscale.scale(x, 1)
        edge_index = self.downscale.edges(1)

        x = self.conv2_enc(x, edge_index)
        x = nn.functional.relu(x)

        x = self.downscale.scale(x, 2)
        edge_index = self.downscale.edges(2)

        x = self.conv3_enc(x, edge_index)
        x = nn.functional.relu(x)

        x = self.downscale.scale(x, 3)
        edge_index = self.downscale.edges(3)

        x = self.conv4_enc(x, edge_index)
        x = nn.functional.relu(x)

        x = self.downscale.scale(x, 4)

        x = x.reshape(-1)
        x = self.fc_enc(x)

        return x

class Decoder(torch.nn.Module):
    def __init__(self, downscale):
        super(Decoder, self).__init__()
        self.downscale = downscale
        # Encoder
        self.conv1_dec = torch_geometric.nn.ChebConv(32, 32, 6)
        self.conv2_dec = torch_geometric.nn.ChebConv(32, 16, 6)
        self.conv3_dec = torch_geometric.nn.ChebConv(16, 16, 6)
        self.conv4_dec = torch_geometric.nn.ChebConv(16, 3, 6)
        self.fc_dec = nn.Linear(8, 704)

    def forward(self, data):

        x = self.fc_dec(data)
        x = x.view(-1, 32)

        x = self.downscale.upscale(x, 4)
        edge_index = self.downscale.edges(3)

        x = self.conv1_dec(x, edge_index)
        x = nn.functional.relu(x)

        x = self.downscale.upscale(x, 3)
        edge_index = self.downscale.edges(2)

        x = self.conv2_dec(x, edge_index)
        x = nn.functional.relu(x)

        x = self.downscale.upscale(x, 2)
        edge_index = self.downscale.edges(1)

        x = self.conv3_dec(x, edge_index)
        x = nn.functional.relu(x)

        x = self.downscale.upscale(x, 1)
        edge_index = self.downscale.edges(0)

        x = self.conv4_dec(x, edge_index)
        #x = nn.functional.relu(x)

        return x

class Downscale():
    def __init__(self, mesh0, mesh1, mesh2, mesh3, mesh4, v1, v2, v3, v4, device):
        addEdges = FaceToEdge(remove_faces=False)
        self.mesh0 = addEdges(mesh0).to(device)
        self.mesh1 = addEdges(mesh1).to(device)
        self.mesh2 = addEdges(mesh2).to(device)
        self.mesh3 = addEdges(mesh3).to(device)
        self.mesh4 = addEdges(mesh4).to(device)
        self.v1 = v1.astype(int)
        self.v2 = v2.astype(int)
        self.v3 = v3.astype(int)
        self.v4 = v4.astype(int)
        self.device=device

    def edges(self, nbr):
        if nbr == 0:
            return self.mesh0.edge_index
        if nbr == 1:
            return self.mesh1.edge_index
        if nbr == 2:
            return self.mesh2.edge_index
        if nbr == 3:
            return self.mesh3.edge_index
        if nbr == 4:
            return self.mesh4.edge_index

    def scale(self, x, nbr):
        newx = []
        if nbr == 1:
            for i in range(self.v1.size):
                newx.append(x[self.v1[i]])

        if nbr == 2:
            for i in range(self.v2.size):
                newx.append(x[self.v2[i]])

        if nbr == 3:
            for i in range(self.v3.size):
                newx.append(x[self.v3[i]])

        if nbr == 4:
            for i in range(self.v4.size):
                newx.append(x[self.v4[i]])

        return torch.stack(newx, axis=0)

    def upscale(self, x, nbr):
        if nbr == 4:
            x = torch_geometric.nn.knn_interpolate(x, self.mesh4.pos, self.mesh3.pos)
            """newx = torch.zeros((self.v3.size, 32))
            for i in range(self.v4.size):
                newx[self.v4[i]] = x[i]
            x=newx.to(self.device)"""
        if nbr == 3:
            x = torch_geometric.nn.knn_interpolate(x, self.mesh3.pos, self.mesh2.pos)
            """newx = torch.zeros((self.v2.size, 32))
            for i in range(self.v3.size):
                newx[self.v3[i]] = x[i]
            x = newx.to(self.device)"""
        if nbr == 2:
            x = torch_geometric.nn.knn_interpolate(x, self.mesh2.pos, self.mesh1.pos)
            """newx = torch.zeros((self.v1.size, 16))
            for i in range(self.v2.size):
                newx[self.v2[i]] = x[i]
            x = newx.to(self.device)"""
        if nbr == 1:
            x = torch_geometric.nn.knn_interpolate(x, self.mesh1.pos, self.mesh0.pos)
            """ newx = torch.zeros((self.mesh0.pos.shape[0], 16))
            for i in range(self.v1.size):
                newx[self.v1[i]] = x[i]
            x = newx.to(self.device)"""
        return x


class Minitest(torch.nn.Module):
    def __init__(self):
        super(Minitest, self).__init__()

    def forward(self, x):
        x = torch_geometric.nn.knn_interpolate(x, x, x)
        return x