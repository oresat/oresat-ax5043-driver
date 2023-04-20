use proc_macro::TokenStream;
use quote::quote;
use syn;

#[proc_macro_derive(Deserialize)]
pub fn deserialize_derive(input: TokenStream) -> TokenStream {
    let ast = syn::parse(input).unwrap();
    impl_deserialize(&ast)
}

fn impl_deserialize(ast: &syn::DeriveInput) -> TokenStream {
    let name = &ast.ident;
    let gen = quote! {
        impl Deserialize<1> for #name {
            fn deserialize(&mut self, data: [u8; 1]) {
                println!("Hello, Macro! My name is {}!", stringify!(#name));
            }
        }
    };
    gen.into()
}

#[proc_macro_derive(Serialize)]
pub fn serialize_derive(input: TokenStream) -> TokenStream {
    let ast = syn::parse(input).unwrap();
    impl_serialize(&ast)
}

fn impl_serialize(ast: &syn::DeriveInput) -> TokenStream {
    let name = &ast.ident;
    let gen = quote! {
        impl Serialzie<1> for #name {
            fn serialize(&self) -> [u8; 1] {
                println!("Hello, Macro! My name is {}!", stringify!(#name));
                [0]
            }
        }
    };
    gen.into()
}
