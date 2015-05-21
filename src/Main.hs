{-# LANGUAGE ForeignFunctionInterface #-}

foreign import ccall "testBullet" testBullet :: IO Int

main = do
    putStrLn "Testing bullet..."
    testBullet
    putStrLn "Done!"