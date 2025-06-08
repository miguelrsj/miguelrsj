const produtos = [
    { id: 1, nome: 'Produto A', preco: 10.0, imagem: 'https://via.placeholder.com/200' },
    { id: 2, nome: 'Produto B', preco: 15.0, imagem: 'https://via.placeholder.com/200' },
    { id: 3, nome: 'Produto C', preco: 20.0, imagem: 'https://via.placeholder.com/200' },
];

const carrinho = [];
const listaProdutos = document.getElementById('produtos');
const listaCarrinho = document.getElementById('lista-carrinho');
const totalDisplay = document.getElementById('total');

function atualizarCarrinho() {
    listaCarrinho.innerHTML = '';
    let total = 0;
    carrinho.forEach((item) => {
        const li = document.createElement('li');
        li.textContent = `${item.nome} - ${item.preco.toFixed(2)} €`;
        listaCarrinho.appendChild(li);
        total += item.preco;
    });
    totalDisplay.textContent = total.toFixed(2);
}

function adicionarAoCarrinho(id) {
    const produto = produtos.find((p) => p.id === id);
    if (produto) {
        carrinho.push(produto);
        atualizarCarrinho();
    }
}

function criarListaProdutos() {
    produtos.forEach((produto) => {
        const div = document.createElement('div');
        div.className = 'produto';
        div.innerHTML = `
            <img src="${produto.imagem}" alt="${produto.nome}">
            <h3>${produto.nome}</h3>
            <p>${produto.preco.toFixed(2)} €</p>
            <button onclick="adicionarAoCarrinho(${produto.id})">Adicionar</button>
        `;
        listaProdutos.appendChild(div);
    });
}

document.addEventListener('DOMContentLoaded', () => {
    criarListaProdutos();
});
