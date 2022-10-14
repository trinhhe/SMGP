import numpy as np
from numpy import genfromtxt
import torch
import torch.nn as nn
from torch_geometric.data import DataLoader
from torch.utils.tensorboard import SummaryWriter
from torch_geometric.io import write_off, read_obj
from torch_geometric.transforms import SamplePoints
from torch_geometric.nn import max_pool_neighbor_x
import os
import argparse

from dataset import FacesDataset
from model import Encoder, Decoder, Downscale

parser = argparse.ArgumentParser(description='Trainings parameters')
parser.add_argument('--train_set', type=str, default="../data/train/",
                    help='path to folder of the dataset for training')
parser.add_argument('--epochs', type=int,
                    help='number of epochs', default=1000)
parser.add_argument('--learningrate', type=float,
                    help='learningrate for the model', default=5e-4)
parser.add_argument('--encoder_ckpt', type=str,
                    help='name of the encoder when saved', default="encoder.pth")
parser.add_argument('--decoder_ckpt', type=str,
                    help='name of the decoder when saved', default="decoder.pth")
args = parser.parse_args()

def train():
    # 4 downscaled meshes
    mesh0 = read_obj("./recon_meshes/mesh0.obj")
    mesh1 = read_obj("./recon_meshes/mesh1.obj")
    mesh2 = read_obj("./recon_meshes/mesh2.obj")
    mesh3 = read_obj("./recon_meshes/mesh3.obj")
    mesh4 = read_obj("./recon_meshes/mesh4.obj")
    v1 = genfromtxt("./recon_meshes/V1.txt")
    v2 = genfromtxt("./recon_meshes/V2.txt")
    v3 = genfromtxt("./recon_meshes/V3.txt")
    v4 = genfromtxt("./recon_meshes/V4.txt")

    downscaler = Downscale(mesh0, mesh1, mesh2, mesh3, mesh4, v1, v2, v3, v4, device)
    writer = SummaryWriter()
    train_dataset = FacesDataset(path=args.train_set)
    loader = DataLoader(train_dataset, batch_size=1)

    encoder = Encoder(downscaler).to(device)
    decoder = Decoder(downscaler).to(device)

    encoder_optimizer = torch.optim.Adam(encoder.parameters(), lr=args.learningrate, weight_decay=5e-4)
    decoder_optimizer = torch.optim.Adam(decoder.parameters(), lr=args.learningrate, weight_decay=5e-4)
    #encoder_optimizer = torch.optim.SGD(encoder.parameters(), lr=8e-3, weight_decay=5e-4, momentum=0.9)
    #decoder_optimizer = torch.optim.SGD(decoder.parameters(), lr=8e-3, weight_decay=5e-4, momentum=0.9)
    step = 0
    for epoch in range(args.epochs):
        print(epoch)
        for batch in loader:
            face = batch.to(device)

            # reset gradient
            encoder_optimizer.zero_grad()
            decoder_optimizer.zero_grad()

            # forward pass
            z = encoder(face)
            reconstruction = decoder(z)

            # loss
            recon_loss = nn.functional.l1_loss(reconstruction, face.pos)

            # backprop
            recon_loss.backward()

            # update parameters
            encoder_optimizer.step()
            decoder_optimizer.step()

            # logging
            writer.add_scalar('L1 loss training', recon_loss, step)
            step += 1

    if not os.path.exists('ckt/'):
        os.makedirs('ckt/')
    torch.save(encoder.state_dict(), args.encoder_ckpt)
    torch.save(decoder.state_dict(), args.decoder_ckpt)



if __name__ == '__main__':
    if torch.cuda.is_available():
        dev = "cuda:0"
    else:
        dev = "cpu"
    device = torch.device(dev)
    train()